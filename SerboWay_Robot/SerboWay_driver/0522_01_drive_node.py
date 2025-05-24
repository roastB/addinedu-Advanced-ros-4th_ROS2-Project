#!/usr/bin/env python3
import time
import math
import numpy as np
import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg      import Int32, Empty
from sensor_msgs.msg   import LaserScan

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        # 1) 토픽 구독
        self.create_subscription(PointStamped, '/robot_pose',    self.pose_cb,         10)
        self.create_subscription(PointStamped, '/robot_front',   self.front_cb,        10)
        for wid in (6, 7):
            self.create_subscription(
                PointStamped, f'/waypoint_{wid}',
                lambda msg, wid=wid: self.wp_cb(wid, msg),
                10
            )
        self.create_subscription(Int32,        '/target_marker', self.target_cb,       10)
        self.create_subscription(PointStamped, '/target_point',  self.target_point_cb,10)
        self.create_subscription(LaserScan,    '/scan',          self.scan_cb,         10)
        self.create_subscription(Empty,        '/arrived',       self.arrived_cb,      10)

        # 2) cmd_vel 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 3) 내부 상태 초기화
        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.marker_world = {
            11:(44,31), 12:(138,31), 13:(144,67), 14:(42,71),
            21:(70,67), 22:(94,67), 23:(118,67),
            31:(144,29),32:(32,67), 97:(91,69), 98:(92,31), 99:(92,15),
            101:(53.4,31.0),102:(62.8,31.0),103:(72.2,31.0),
            104:(81.6,31.0),105:(91.0,31.0),106:(100.4,31.0),
            107:(109.8,31.0),108:(119.2,31.0),109:(128.6,31.0),
            110:(133.8,67.4),111:(123.6,67.8),112:(113.4,68.2),
            113:(103.2,68.6),114:(93.0,69.0), 115:(82.8,69.4),
            116:(72.6,69.8),117:(62.4,70.2),118:(52.2,70.6),
            119:(139.2,38.2),120:(140.4,45.4),121:(141.6,52.6),
            122:(142.8,59.8),123:(43.6,39.0),124:(43.2,47.0),
            125:(42.8,55.0),126:(42.4,63.0),
        }
        self.target      = None
        self.start_pose  = None
        self.last_target = None
        self.d_f = self.d_l = self.d_r = float('nan')

        # 4) 파라미터 설정 (m, rad, s 단위)
        self.fixed_omega      = 0.1      # rad/s
        self.turn_interval    = 0.03     # s
        self.pause_interval   = 0.07     # s
        self.drive_speed      = 0.1      # m/s
        self.angle_tolerance  = 3.0      # deg
        self.cont_accum       = 0.0

        self.obs_detect_thresh  = 0.15   # m
        self.obs_clear_thresh   = 0.25   # m
        self.side_detect_thresh = 0.15   # m
        self.drive_obs_time     = 0.3    # s
        self.surf_drive_time    = 0.3    # s
        self.too_close_thresh   = 0.10   # m

        self.mini_deg         = 20.0     # deg
        self.mini_rad         = math.radians(self.mini_deg)

        self.state        = 'IDLE'
        self.rotation_dir = 0.0
        self.accum_time   = 0.0
        self.prev_time    = time.time()

        # 5) 제어 루프 타이머 (200Hz)
        self.create_timer(1/200.0, self.control_loop)

    # === 콜백들 ===
    def pose_cb(self, msg: PointStamped):
        self.wx_r = msg.point.x / 100.0
        self.wy_r = msg.point.y / 100.0

    def front_cb(self, msg: PointStamped):
        self.wx_f = msg.point.x / 100.0
        self.wy_f = msg.point.y / 100.0

    def wp_cb(self, wid, msg: PointStamped):
        self.marker_world[wid] = (msg.point.x, msg.point.y)

    def target_cb(self, msg: Int32):
        tid = msg.data
        if tid in self.marker_world and self.wx_r is not None:
            tx_cm, ty_cm = self.marker_world[tid]
            tx, ty = tx_cm/100.0, ty_cm/100.0
            self.start_pose  = (self.wx_r, self.wy_r)
            self.last_target = tid
            self.target      = (tx, ty)
            self.state       = 'IDLE'
            self.accum_time  = 0.0
            for a in ('drive_obs_cleared','surf_obs_detected','surf_clear_time',
                      '_rotate_obs_start','_rotate_surf_start','_rotate3_start','third_obs_detected'):
                if hasattr(self, a):
                    delattr(self, a)
            self.get_logger().info(f"▶ target set {tid} at ({tx:.2f},{ty:.2f}) m")
            self.cont_accum = 0.0

    def target_point_cb(self, msg: PointStamped):
        if self.wx_r is None:
            return
        self.start_pose = (self.wx_r, self.wy_r)
        tx, ty = msg.point.x/100.0, msg.point.y/100.0
        self.target = (tx, ty)
        if abs(tx-1.44)<0.03 and abs(ty-0.29)<0.03:
            self.last_target = 31
        elif abs(tx-0.32)<0.03 and abs(ty-0.67)<0.03:
            self.last_target = 32
        else:
            self.last_target = None
        self.state, self.accum_time = 'IDLE', 0.0
        for a in ('drive_obs_cleared','surf_obs_detected','surf_clear_time',
                  '_rotate_obs_start','_rotate_surf_start','_rotate3_start','third_obs_detected'):
            if hasattr(self, a):
                delattr(self, a)
        self.get_logger().info(f"▶ target_point ({tx:.2f},{ty:.2f}) m, last_target={self.last_target}")
        self.cont_accum = 0.0

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        N, mid = len(ranges), len(ranges)//2
        f, l, r = (mid+N//2)%N, (mid-N//4)%N, (mid+N//4)%N
        span = int(math.radians(5)/msg.angle_increment)
        def md(i):
            w = np.concatenate([ranges[i-span:i], ranges[i:i+span+1]])
            v = w[(w>0)&~np.isinf(w)]
            return float(v.min()) if v.size else float('nan')
        self.d_f, self.d_l, self.d_r = md(f), md(l), md(r)

    def arrived_cb(self, msg: Empty):
        if self.target is None:
            return
        if self.last_target in (31,32):
            self.get_logger().info("▶ arrived → FINAL_TURN")
            self.cmd_pub.publish(Twist())
            self.state, self.accum_time = 'FINAL_TURN', 0.0
            self.ft_accum_cont = 0.0
            yaw = math.atan2(self.wy_f - self.wy_r, self.wx_f - self.wx_r)
            desired = math.pi if self.last_target==31 else 0.0
            abs_e0 = abs((desired - yaw + math.pi)%(2*math.pi) - math.pi)
            self.ft_threshold = max(abs_e0 - self.mini_rad, 0.0)
        else:
            self.get_logger().info("▶ arrived → STOP")
            self.cmd_pub.publish(Twist())
            self.state, self.target, self.last_target = 'IDLE', None, None

    def control_loop(self):
        now = time.time()
        dt  = now - self.prev_time
        self.prev_time = now
        twist = Twist()

        # --- FINAL_TURN 처리 ---
        if self.state == 'FINAL_TURN':
            yaw     = math.atan2(self.wy_f - self.wy_r, self.wx_f - self.wx_r)
            desired = math.pi if self.last_target == 31 else 0.0
            error   = (desired - yaw + math.pi) % (2*math.pi) - math.pi
            abs_err = abs(error)

            # 도달 종료: 목표 완전 삭제 후 정지
            if abs_err < math.radians(self.angle_tolerance):
                self.state       = 'IDLE'
                self.target      = None
                self.last_target = None
                self.cmd_pub.publish(Twist())
                return

            # continuous 회전량 = abs_err - mini_rad
            cont_threshold = max(abs_err - self.mini_rad, 0.0)

            # 1) continuous
            if cont_threshold > 0 and self.ft_accum_cont < cont_threshold:
                self.ft_accum_cont += abs(self.fixed_omega * dt)
                twist.angular.z = self.fixed_omega 
                self.cmd_pub.publish(twist)
                return

            # 2) 마지막 mini_rad bang–bang
            if self.accum_time < self.turn_interval:
                twist.angular.z = self.fixed_omega 
            elif self.accum_time < self.turn_interval + self.pause_interval:
                twist.angular.z = 0.0
            else:
                self.accum_time = 0.0
            self.accum_time += dt
            self.cmd_pub.publish(twist)
            return

        # --- pose·target 유효성 검사 ---
        if None in (self.wx_r, self.wy_r, self.wx_f, self.wy_f) or self.target is None:
            self.cmd_pub.publish(Twist())
            return

        # --- 장애물 회피 FSM ---
        if self.state == 'DRIVING' and not math.isnan(self.d_f) and self.d_f < self.obs_detect_thresh:
            self.state, self.accum_time = 'CONFIRM_OBS', 0.0
            return

        if self.state == 'CONFIRM_OBS':
            if not math.isnan(self.d_f) and self.d_f < self.obs_detect_thresh:
                self.accum_time += dt
                if self.accum_time >= 1.0:
                    self.state, self.accum_time = 'STOP_OBS', 0.0
            else:
                self.state, self.accum_time = 'DRIVING', 0.0
            self.cmd_pub.publish(Twist())
            return

        if self.state == 'STOP_OBS':
            self.accum_time += dt
            if self.accum_time >= self.pause_interval:
                self._obs_heading0 = math.atan2(self.wy_f - self.wy_r, self.wx_f - self.wx_r)
                self.rotation_dir  = 1.0 if self.d_l > self.d_r else -1.0
                self.state, self.accum_time = 'ROTATING_OBS', 0.0
            self.cmd_pub.publish(Twist())
            return

        if self.state == 'ROTATING_OBS':
            heading = math.atan2(self.wy_f - self.wy_r, self.wx_f - self.wx_r)
            if not hasattr(self, '_rotate_obs_start'):
                self._rotate_obs_start, self.accum_time = heading, 0.0
            delta = (heading - self._rotate_obs_start + math.pi) % (2*math.pi) - math.pi
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
                actual = abs(math.degrees(delta))
                if abs(actual - 90.0) <= 2.0:
                    self.get_logger().info(f"▷ ROTATING_OBS done — {actual:.1f}°")
                    delattr(self, '_rotate_obs_start')
                    self.state, self.accum_time = 'DRIVE_OBS', 0.0
                else:
                    self.get_logger().info(f"▷ ROT_OBS off — {actual-90.0:.1f}°")
            return

        # --- 기본 FSM: IDLE → ROTATING → PAUSE → DRIVING ---
        tx, ty = self.target
        fv = (self.wx_f - self.wx_r, self.wy_f - self.wy_r)
        tv = (tx - self.wx_r,        ty - self.wy_r)
        err = math.degrees(math.atan2(fv[0]*tv[1] - fv[1]*tv[0],
                                      fv[0]*tv[0] + fv[1]*tv[1]))

        if self.state == 'IDLE':
            if abs(err) >= self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0, err)
                self.accum_time   = 0.0
                self.state        = 'ROTATING'
                self.cont_accum   = 0.0
            else:
                self.state = 'DRIVING'

        elif self.state == 'ROTATING':
            abs_err = abs(err)
            cont_thr = max(abs_err - self.mini_deg, 0.0)
            if cont_thr > 0 and self.cont_accum < cont_thr:
                twist.angular.z = math.copysign(self.fixed_omega, err)
                self.cont_accum += abs(self.fixed_omega * dt) * (180/math.pi)
                self.cmd_pub.publish(twist)
                return
            if self.accum_time < self.turn_interval:
                twist.angular.z = math.copysign(self.fixed_omega, err)
            elif self.accum_time < self.turn_interval + self.pause_interval:
                twist.angular.z = 0.0
            else:
                self.accum_time = 0.0
            self.accum_time += dt
            if abs_err < self.angle_tolerance:
                self.state = 'DRIVING'
                self.accum_time = 0.0
                self.cmd_pub.publish(Twist())
                return
            self.cmd_pub.publish(twist)
            return

        elif self.state == 'PAUSE':
            self.accum_time += dt
            if self.accum_time >= self.pause_interval:
                self.accum_time = 0.0
                if abs(err) >= self.angle_tolerance:
                    self.rotation_dir = math.copysign(1.0, err)
                    self.state = 'ROTATING'
                else:
                    self.state = 'DRIVING'
            self.cmd_pub.publish(Twist())
            return

        elif self.state == 'DRIVING':
            if abs(err) >= self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0, err)
                self.state = 'ROTATING'
                self.accum_time = 0.0
                twist.angular.z = self.rotation_dir * self.fixed_omega
            else:
                twist.linear.x = self.drive_speed
            self.cmd_pub.publish(twist)
            return

def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
