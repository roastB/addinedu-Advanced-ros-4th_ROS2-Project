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

        # state
        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.marker_world = {}
        self.target = None
        self.d_f = self.d_l = self.d_r = float('nan')

        # drive params
        self.fixed_omega     = 0.1    # rad/s
        self.turn_interval   = 0.05   # s
        self.pause_interval  = 0.07   # s
        self.drive_speed     = 0.1    # m/s
        self.angle_tolerance = 5.0    # deg
        self.dist_tolerance  = 1.0    # m

        # obstacle params
        self.obs_detect_thresh = 0.20  # m
        self.obs_clear_thresh  = 0.25  # m
        # split extra drive times
        self.drive_obs_time   = 1.15   # s 첫 회피 전진
        self.surf_drive_time  = 0.50   # s 두 번째 회전 후 전진

        # FSM
        self.state        = 'IDLE'
        self.rotation_dir = 0.0
        self.accum_time   = 0.0
        self.prev_time    = time.time()
        self._obs_heading0 = 0.0

        # timer
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
            self.get_logger().info(f"▶ target set to {tid}")
            self.target     = self.marker_world[tid]
            self.state      = 'IDLE'
            self.accum_time = 0.0
            for a in ('confirm_start','drive_obs_cleared','surf_obs_detected','surf_clear_time'):
                if hasattr(self, a): delattr(self, a)

    def scan_cb(self, msg: LaserScan):
        r = np.array(msg.ranges)
        N = len(r)
        mid        = N//2
        front_idx  = (mid + N//2) % N
        left_idx   = (mid - N//4) % N
        right_idx  = (mid + N//4) % N
        span = int(math.radians(5)/msg.angle_increment)
        def md(idx):
            w = np.concatenate([r[idx-span:idx], r[idx:idx+span+1]])
            v = w[(w>0)&~np.isinf(w)]
            return float(v.min()) if v.size else float('nan')
        self.d_f = md(front_idx)
        self.d_l = md(left_idx)
        self.d_r = md(right_idx)

    def control_loop(self):
        now = time.time()
        dt  = now - self.prev_time
        self.prev_time = now
        twist = Twist()

        # wait for data
        if None in (self.wx_r,self.wy_r,self.wx_f,self.wy_f) or self.target is None:
            self.cmd_pub.publish(twist)
            return

        # compute error
        tx,ty = self.target
        fv = (self.wx_f - self.wx_r, self.wy_f - self.wy_r)
        tv = (tx - self.wx_r,        ty - self.wy_r)
        dist = math.hypot(*tv)
        err  = math.degrees(math.atan2(
            fv[0]*tv[1] - fv[1]*tv[0],
            fv[0]*tv[0] + fv[1]*tv[1]
        ))

        # arrival
        if dist <= self.dist_tolerance:
            self.get_logger().info("■ arrived — stopping")
            self.target = None
            self.state  = 'IDLE'
            self.cmd_pub.publish(Twist())
            return

        # --- obstacle avoidance ---
        # 1) DRIVING → CONFIRM_OBS
        if self.state=='DRIVING' and not math.isnan(self.d_f) and self.d_f<self.obs_detect_thresh:
            self.state      = 'CONFIRM_OBS'
            self.accum_time = 0.0
            self.get_logger().info("▷ possible obstacle — CONFIRM_OBS")
            return

        # 2) CONFIRM_OBS
        if self.state=='CONFIRM_OBS':
            if not math.isnan(self.d_f) and self.d_f<self.obs_detect_thresh:
                self.accum_time+=dt
                if self.accum_time>=1.0:
                    self.state      = 'STOP_OBS'
                    self.accum_time = 0.0
                    self.get_logger().info("▷ confirmed obstacle — STOP_OBS")
            else:
                self.state      = 'DRIVING'
                self.accum_time = 0.0
                self.get_logger().info("▷ false alarm — back to DRIVING")
            self.cmd_pub.publish(Twist())
            return

        # 3) STOP_OBS → ROTATING_OBS
        if self.state=='STOP_OBS':
            self.accum_time+=dt
            if self.accum_time>=self.pause_interval:
                self._obs_heading0 = math.atan2(self.wy_f-self.wy_r, self.wx_f-self.wx_r)
                self.rotation_dir  = +1.0 if self.d_l>self.d_r else -1.0
                self.state         = 'ROTATING_OBS'
                self.accum_time    = 0.0
                self.get_logger().info("▷ start ROTATING_OBS")
            self.cmd_pub.publish(Twist())
            return

        # 4) ROTATING_OBS
        if self.state=='ROTATING_OBS':
            heading = math.atan2(self.wy_f-self.wy_r, self.wx_f-self.wx_r)
            delta   = (heading-self._obs_heading0+math.pi)%(2*math.pi)-math.pi
            if abs(delta)<math.pi/2-0.05:
                if self.accum_time<self.turn_interval:
                    twist.angular.z = self.rotation_dir*self.fixed_omega
                elif self.accum_time<self.turn_interval+self.pause_interval:
                    pass
                else:
                    self.accum_time=0.0
                self.accum_time+=dt
                self.cmd_pub.publish(twist)
            else:
                self.state      = 'DRIVE_OBS'
                self.accum_time = 0.0
                self.get_logger().info("▷ ROTATING_OBS done — DRIVE_OBS")
            return

        # 5) DRIVE_OBS
        if self.state=='DRIVE_OBS':
            if not hasattr(self,'drive_obs_cleared'):
                if not math.isnan(self.d_f) and self.d_f>=self.obs_detect_thresh:
                    self.drive_obs_cleared = now
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return
            if now-self.drive_obs_cleared>=self.drive_obs_time:
                self.state         = 'ROTATE_SURFACE'
                self.accum_time    = 0.0
                self.rotation_dir *= -1
                self._obs_heading0 = math.atan2(self.wy_f-self.wy_r, self.wx_f-self.wx_r)
                self.get_logger().info("▷ DRIVE_OBS done — ROTATE_SURFACE")
                delattr(self,'drive_obs_cleared')
            else:
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
            return

        # 6) ROTATE_SURFACE
        if self.state=='ROTATE_SURFACE':
            heading = math.atan2(self.wy_f-self.wy_r, self.wx_f-self.wx_r)
            delta   = (heading-self._obs_heading0+math.pi)%(2*math.pi)-math.pi
            if abs(delta)<math.pi/2-0.05:
                if self.accum_time<self.turn_interval:
                    twist.angular.z=self.rotation_dir*self.fixed_omega
                elif self.accum_time<self.turn_interval+self.pause_interval:
                    pass
                else:
                    self.accum_time=0.0
                self.accum_time+=dt
                self.cmd_pub.publish(twist)
            else:
                self.state      = 'SURFACE_DRIVE'
                self.accum_time = 0.0
                self.get_logger().info("▷ ROTATE_SURFACE done — SURFACE_DRIVE")
            return

        # 7) SURFACE_DRIVE
        if self.state=='SURFACE_DRIVE':
            side = self.d_r if self.rotation_dir<0 else self.d_l
            if not hasattr(self,'surf_obs_detected'):
                if side<=self.obs_detect_thresh:
                    self.surf_obs_detected = now
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return
            if not hasattr(self,'surf_clear_time'):
                if side>=self.obs_clear_thresh:
                    self.surf_clear_time = now
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return
            # 추가 전진: surf_drive_time 사용
            if now-self.surf_clear_time<self.surf_drive_time:
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return
            self.get_logger().info("▷ SURFACE_DRIVE done — resume")
            self.state='IDLE'
            for a in ('surf_obs_detected','surf_clear_time'):
                if hasattr(self,a): delattr(self,a)
            self.cmd_pub.publish(Twist())
            return

        # --- original driving FSM ---
        if self.state == 'IDLE':
            if abs(err)>=self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0,err)
                self.accum_time   = 0.0
                self.state        = 'ROTATING'
            else:
                self.state        = 'DRIVING'

        elif self.state == 'ROTATING':
            if abs(err)<self.angle_tolerance:
                self.state      = 'DRIVING'
                self.accum_time = 0.0
                self.cmd_pub.publish(Twist())
                return
            if self.accum_time<self.turn_interval:
                twist.angular.z = self.rotation_dir*self.fixed_omega
            elif self.accum_time<self.turn_interval+self.pause_interval:
                pass
            else:
                self.accum_time=0.0
            self.accum_time+=dt
            self.cmd_pub.publish(twist)
            return

        elif self.state == 'PAUSE':
            self.accum_time+=dt
            if self.accum_time>=self.pause_interval:
                self.accum_time=0.0
                if abs(err)>=self.angle_tolerance:
                    self.rotation_dir = math.copysign(1.0,err)
                    self.state        = 'ROTATING'
                else:
                    self.state        = 'DRIVING'
            self.cmd_pub.publish(Twist())
            return

        elif self.state == 'DRIVING':
            if abs(err)>=self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0,err)
                self.state        = 'ROTATING'
                self.accum_time   = 0.0
                twist.angular.z   = self.rotation_dir*self.fixed_omega
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
