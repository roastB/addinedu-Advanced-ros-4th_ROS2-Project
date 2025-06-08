#!/usr/bin/env python3
import time
import math
import numpy as np
import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg      import Int32
from sensor_msgs.msg   import LaserScan

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        # subscriptions
        self.create_subscription(PointStamped, '/robot_pose',  self.pose_cb,   10)
        self.create_subscription(PointStamped, '/robot_front', self.front_cb,  10)
        for wid in (6,7,8):
            self.create_subscription(
                PointStamped, f'/waypoint_{wid}',
                lambda msg, wid=wid: self.wp_cb(wid, msg),
                10
            )
        self.create_subscription(Int32, '/target_marker', self.target_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # robot & target state
        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.marker_world = {}
        self.target = None
        self.start_pose = None
        self.last_target = None
        # LiDAR distances
        self.d_f = self.d_l = self.d_r = float('nan')

        # --- drive parameters ---
        self.fixed_omega      = 0.1    # rad/s 회전 속도
        self.turn_interval    = 0.05   # s 미니 회전 시간
        self.pause_interval   = 0.07   # s 미니 회전 중 멈춤 시간
        self.drive_speed      = 0.1    # m/s 전진 속도
        self.angle_tolerance  = 5.0    # deg 목표 방향 허용 오차
        self.dist_tolerance   = 1.0    # m 목표 도달 허용 거리

        # --- obstacle avoidance parameters ---
        self.obs_detect_thresh  = 0.15
        self.obs_clear_thresh   = 0.25
        self.side_detect_thresh = 0.15
        self.drive_obs_time     = 0.3
        self.surf_drive_time    = 0.3
        self.too_close_thresh   = 0.10

        # FSM state
        self.state         = 'IDLE'
        self.rotation_dir  = 0.0
        self.accum_time    = 0.0
        self.prev_time     = time.time()
        self._obs_heading0 = 0.0

        # timer (200Hz)
        self.create_timer(1/200.0, self.control_loop)

    def pose_cb(self, msg: PointStamped):
        self.wx_r, self.wy_r = msg.point.x, msg.point.y

    def front_cb(self, msg: PointStamped):
        self.wx_f, self.wy_f = msg.point.x, msg.point.y

    def wp_cb(self, wid, msg: PointStamped):
        self.marker_world[wid] = (msg.point.x, msg.point.y)

    def target_cb(self, msg: Int32):
        tid = msg.data
        if tid in self.marker_world and self.wx_r is not None:
            self.start_pose = (self.wx_r, self.wy_r)
            self.last_target = tid
            self.target = self.marker_world[tid]
            self.state  = 'IDLE'
            self.accum_time = 0.0
            # clear any previous flags
            for a in ('drive_obs_cleared','surf_obs_detected','surf_clear_time',
                      '_rotate_obs_start','_rotate_surf_start','_rotate3_start','third_obs_detected'):
                if hasattr(self, a):
                    delattr(self, a)
            self.get_logger().info(f"▶ target set to {tid}")

    def scan_cb(self, msg: LaserScan):
        r = np.array(msg.ranges)
        N = len(r); mid = N//2
        front_idx = (mid + N//2) % N
        left_idx  = (mid - N//4) % N
        right_idx = (mid + N//4) % N
        span = int(math.radians(5)/msg.angle_increment)
        def md(idx):
            w = np.concatenate([r[idx-span:idx], r[idx:idx+span+1]])
            v = w[(w>0)&~np.isinf(w)]
            return float(v.min()) if v.size else float('nan')
        self.d_f, self.d_l, self.d_r = md(front_idx), md(left_idx), md(right_idx)

    def control_loop(self):
        now = time.time()
        dt  = now - self.prev_time
        self.prev_time = now
        twist = Twist()

        # 0) wait for pose & target
        if None in (self.wx_r, self.wy_r, self.wx_f, self.wx_f) or self.target is None:
            self.cmd_pub.publish(twist)
            return

        # 1) compute error to target
        tx, ty = self.target
        fv = (self.wx_f - self.wx_r, self.wy_f - self.wy_r)
        tv = (tx - self.wx_r,        ty - self.wy_r)
        dist = math.hypot(*tv)
        err  = math.degrees(math.atan2(
            fv[0]*tv[1] - fv[1]*tv[0],
            fv[0]*tv[0] + fv[1]*tv[1]
        ))

        # 2) arrival check
        if dist <= self.dist_tolerance:
            self.get_logger().info("■ arrived — stopping")
            self.state  = 'IDLE'
            self.target = None
            self.cmd_pub.publish(Twist())
            return

        # 3) OBSTACLE AVOIDANCE START
        if self.state == 'DRIVING' and not math.isnan(self.d_f) and self.d_f < self.obs_detect_thresh:
            self.state      = 'CONFIRM_OBS'
            self.accum_time = 0.0
            self.get_logger().info("▷ possible obstacle — CONFIRM_OBS")
            return

        # 4) CONFIRM_OBS
        if self.state == 'CONFIRM_OBS':
            if not math.isnan(self.d_f) and self.d_f < self.obs_detect_thresh:
                self.accum_time += dt
                if self.accum_time >= 1.0:
                    self.state      = 'STOP_OBS'
                    self.accum_time = 0.0
                    self.get_logger().info("▷ confirmed obstacle — STOP_OBS")
            else:
                self.state      = 'DRIVING'
                self.accum_time = 0.0
                self.get_logger().info("▷ false alarm — back to DRIVING")
            self.cmd_pub.publish(Twist())
            return

        # 5) STOP_OBS
        if self.state == 'STOP_OBS':
            self.accum_time += dt
            if self.accum_time >= self.pause_interval:
                self._obs_heading0 = math.atan2(self.wy_f - self.wy_r,
                                                self.wx_f - self.wx_r)
                self.rotation_dir  = +1.0 if self.d_l > self.d_r else -1.0
                self.state         = 'ROTATING_OBS'
                self.accum_time    = 0.0
                self.get_logger().info("▷ start ROTATING_OBS")
            self.cmd_pub.publish(Twist())
            return

        # 6) ROTATING_OBS (90° correction with mini-rotation)
        if self.state == 'ROTATING_OBS':
            heading = math.atan2(self.wy_f - self.wy_r,
                                 self.wx_f - self.wx_r)
            if not hasattr(self, '_rotate_obs_start'):
                self._rotate_obs_start = heading
                self.accum_time        = 0.0
            delta = (heading - self._rotate_obs_start + math.pi) % (2*math.pi) - math.pi
            # mini rotation cycle
            if abs(delta) < math.pi/2 - 0.05:
                if self.accum_time < self.turn_interval:
                    twist.angular.z = self.rotation_dir * self.fixed_omega
                elif self.accum_time < self.turn_interval + self.pause_interval:
                    pass
                else:
                    self.accum_time = 0.0
                self.accum_time += dt
                self.cmd_pub.publish(twist)
            else:
                actual_deg = abs(math.degrees(delta))
                error_deg  = actual_deg - 90.0
                if abs(error_deg) <= 2.0:
                    self.get_logger().info(f"▷ ROTATING_OBS done — actual {actual_deg:.1f}°")
                    delattr(self, '_rotate_obs_start')
                    self.state      = 'DRIVE_OBS'
                    self.accum_time = 0.0
                else:
                    self.get_logger().info(f"▷ ROTATING_OBS off by {error_deg:.1f}°, correcting")
                    # continue correction in same state
            return

        # 7) DRIVE_OBS
        if self.state == 'DRIVE_OBS':
            side_dist = self.d_r if self.rotation_dir > 0 else self.d_l
            if not hasattr(self, 'drive_obs_cleared'):
                if not math.isnan(side_dist) and side_dist >= self.obs_clear_thresh:
                    self.drive_obs_cleared = now
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return
            if now - self.drive_obs_cleared >= self.drive_obs_time:
                self.state         = 'ROTATE_SURFACE'
                self.accum_time    = 0.0
                self.rotation_dir *= -1
                self._obs_heading0 = math.atan2(self.wy_f - self.wy_r,
                                                self.wx_f - self.wx_r)
                self.get_logger().info("▷ DRIVE_OBS done — ROTATE_SURFACE")
                delattr(self, 'drive_obs_cleared')
            else:
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
            return

        # 8) ROTATE_SURFACE (90° correction with mini-rotation)
        if self.state == 'ROTATE_SURFACE':
            heading = math.atan2(self.wy_f - self.wy_r,
                                 self.wx_f - self.wx_r)
            if not hasattr(self, '_rotate_surf_start'):
                self._rotate_surf_start = heading
                self.accum_time         = 0.0
            delta = (heading - self._rotate_surf_start + math.pi) % (2*math.pi) - math.pi
            if abs(delta) < math.pi/2 - 0.05:
                if self.accum_time < self.turn_interval:
                    twist.angular.z = self.rotation_dir * self.fixed_omega
                elif self.accum_time < self.turn_interval + self.pause_interval:
                    pass
                else:
                    self.accum_time = 0.0
                self.accum_time += dt
                self.cmd_pub.publish(twist)
            else:
                actual_deg = abs(math.degrees(delta))
                error_deg  = actual_deg - 90.0
                if abs(error_deg) <= 2.0:
                    self.get_logger().info(f"▷ ROTATE_SURFACE done — actual {actual_deg:.1f}°")
                    delattr(self, '_rotate_surf_start')
                    self.state      = 'SURFACE_DRIVE'
                    self.accum_time = 0.0
                else:
                    self.get_logger().info(f"▷ ROTATE_SURFACE off by {error_deg:.1f}°, correcting")
            return


        # 9) SURFACE_DRIVE (unchanged)
        if self.state == 'SURFACE_DRIVE':
            side = self.d_r if self.rotation_dir < 0 else self.d_l
            if not hasattr(self, 'surf_obs_detected'):
                if side <= self.side_detect_thresh:
                    self.surf_obs_detected = now
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return
            if not hasattr(self, 'surf_clear_time'):
                if side >= self.obs_clear_thresh:
                    self.surf_clear_time = now
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return
            if side < self.too_close_thresh:
                self.state        = 'SURFACE_ADJUST'
                self.adj_heading0 = math.atan2(self.wy_f - self.wy_r,
                                               self.wx_f - self.wx_r)
                self.adj_dir      = -self.rotation_dir
                self.accum_time   = 0.0
                return
            if now - self.surf_clear_time < self.surf_drive_time:
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return
            self.cmd_pub.publish(Twist())
            self.get_logger().info("▷ SURFACE_DRIVE done — start ROTATE_THIRD")
            self.state         = 'ROTATE_THIRD'
            self.accum_time    = 0.0
            self._obs_heading0 = math.atan2(self.wy_f - self.wy_r,
                                            self.wx_f - self.wx_r)
            return

        # 10) SURFACE_ADJUST (unchanged)
        if self.state == 'SURFACE_ADJUST':
            if self.accum_time < self.turn_interval:
                twist.angular.z = self.adj_dir * self.fixed_omega
                self.accum_time += dt
                self.cmd_pub.publish(twist)
            else:
                self.state      = 'SURFACE_DRIVE'
                self.accum_time = 0.0
            return

        # 11) ROTATE_THIRD (90° correction with mini-rotation)
        if self.state == 'ROTATE_THIRD':
            heading = math.atan2(self.wy_f - self.wy_r,
                                 self.wx_f - self.wx_r)
            if not hasattr(self, '_rotate3_start'):
                self._rotate3_start = heading
                self.accum_time      = 0.0
            delta = (heading - self._rotate3_start + math.pi) % (2*math.pi) - math.pi
            if abs(delta) < math.pi/2 - 0.05:
                if self.accum_time < self.turn_interval:
                    twist.angular.z = self.rotation_dir * self.fixed_omega
                elif self.accum_time < self.turn_interval + self.pause_interval:
                    pass
                else:
                    self.accum_time = 0.0
                self.accum_time += dt
                self.cmd_pub.publish(twist)
            else:
                actual_deg = abs(math.degrees(delta))
                error_deg  = actual_deg - 90.0
                if abs(error_deg) <= 2.0:
                    self.get_logger().info(f"▷ ROTATE_THIRD done — actual {actual_deg:.1f}°")
                    delattr(self, '_rotate3_start')
                    self.state      = 'THIRD_DRIVE'
                    self.accum_time = 0.0
                else:
                    self.get_logger().info(f"▷ ROTATE_THIRD off by {error_deg:.1f}°, correcting")
            return

        # 12) THIRD_DRIVE (with boolean flag and line detection)
        if self.state == 'THIRD_DRIVE':
            # initialize once
            if not hasattr(self, 'third_obs_detected'):
                self.third_obs_detected = False
                self.accum_time          = 0.0
            # side detection
            side = self.d_r if self.rotation_dir < 0 else self.d_l
            if not self.third_obs_detected:
                if side <= self.side_detect_thresh:
                    self.third_obs_detected = True
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return
            # after detection, go straight until line
            twist.linear.x = self.drive_speed
            self.cmd_pub.publish(twist)
            # line detection
            Sx, Sy = self.start_pose
            Tx, Ty = self.target
            Pr, Pc = self.wx_r, self.wy_r
            st_x, st_y = Tx - Sx, Ty - Sy
            sp_x, sp_y = Pr - Sx, Pc - Sy
            den = st_x*st_x + st_y*st_y
            u = (sp_x*st_x + sp_y*st_y) / den if den>0 else 0.0
            cross = abs(sp_x*st_y - sp_y*st_x) / math.sqrt(den) if den>0 else float('inf')
            if u >= 1.0 or cross < 2.0:
                self.get_logger().info("▷ THIRD_DRIVE line hit — stopping")
                self.state = 'IDLE'
                self.cmd_pub.publish(Twist())
            return

        # 13) original driving FSM: IDLE → ROTATING → PAUSE → DRIVING
        if self.state == 'IDLE':
            if abs(err) >= self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0, err)
                self.accum_time   = 0.0
                self.state        = 'ROTATING'
            else:
                self.state        = 'DRIVING'

        elif self.state == 'ROTATING':
            if abs(err) < self.angle_tolerance:
                self.state        = 'DRIVING'
                self.accum_time   = 0.0
                self.cmd_pub.publish(Twist())
                return
            new_dir = math.copysign(1.0, err)
            if new_dir != self.rotation_dir:
                self.rotation_dir = new_dir
                self.accum_time   = 0.0
            if self.accum_time < self.turn_interval:
                twist.angular.z = self.rotation_dir * self.fixed_omega
            elif self.accum_time < self.turn_interval + self.pause_interval:
                pass
            else:
                self.accum_time = 0.0
            self.accum_time += dt
            self.cmd_pub.publish(twist)
            return

        elif self.state == 'PAUSE':
            self.accum_time += dt
            if self.accum_time >= self.pause_interval:
                self.accum_time = 0.0
                if abs(err) >= self.angle_tolerance:
                    self.rotation_dir = math.copysign(1.0, err)
                    self.state        = 'ROTATING'
                else:
                    self.state        = 'DRIVING'
            self.cmd_pub.publish(Twist())
            return

        elif self.state == 'DRIVING':
            if abs(err) >= self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0, err)
                self.state        = 'ROTATING'
                self.accum_time   = 0.0
                twist.angular.z   = self.rotation_dir * self.fixed_omega
            else:
                twist.linear.x    = self.drive_speed

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
