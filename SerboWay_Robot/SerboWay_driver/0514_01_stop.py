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
        # 1) 비전 토픽 구독
        self.create_subscription(PointStamped, '/robot_pose',  self.pose_cb,   10)
        self.create_subscription(PointStamped, '/robot_front', self.front_cb,  10)
        for wid in (6,7,8):
            self.create_subscription(
                PointStamped,
                f'/waypoint_{wid}',
                lambda msg, wid=wid: self.wp_cb(wid, msg),
                10
            )
        self.create_subscription(Int32, '/target_marker', self.target_cb, 10)

        # 2) LiDAR 토픽 구독
        self.scan_ranges = None
        self.declare_parameter('window_deg', 5.0)
        self.win_rad = math.radians(self.get_parameter('window_deg').value)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # 3) cmd_vel 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 상태 변수
        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.marker_world = {}
        self.target = None
        self.d_f = self.d_b = self.d_l = self.d_r = float('nan')

        # 제어 파라미터
        self.fixed_omega     = 0.1    # rad/s 회전 속도
        self.turn_interval   = 0.03   # s 회전 시간
        self.pause_interval  = 0.1  # s 대기 시간
        self.drive_speed     = 0.05    # m/s 전진 속도
        self.angle_tolerance = 3.0    # deg
        self.dist_tolerance  = 1.0    # m (목표 도달 기준)
        self.obs_threshold   = 0.2    # m (전방 정지 기준 20 cm)

        # 상태 머신 변수
        self.state        = 'IDLE'    # IDLE, ROTATING, PAUSE, DRIVING
        self.rotation_dir = 0.0
        self.accum_time   = 0.0
        self.prev_time    = time.time()

        # 제어 루프 타이머 (200 Hz)
        self.create_timer(1/200.0, self.control_loop)

    # ── 콜백들 ──
    def pose_cb(self, msg):
        self.wx_r, self.wy_r = msg.point.x, msg.point.y

    def front_cb(self, msg):
        self.wx_f, self.wy_f = msg.point.x, msg.point.y

    def wp_cb(self, wid, msg):
        self.marker_world[wid] = (msg.point.x, msg.point.y)

    def target_cb(self, msg):
        tid = msg.data
        if tid in self.marker_world and self.wx_r is not None:
            self.get_logger().info(f"▶ target set to {tid}")
            self.target     = self.marker_world[tid]
            self.state      = 'IDLE'
            self.accum_time = 0.0

    def scan_cb(self, msg: LaserScan):
        r = np.array(msg.ranges)
        N   = len(r)
        mid = N//2             # 센서 배열상의 “앞”
        back  = (mid + N//2) % N
        left  = (mid - N//4) % N  # 화면과 반대로
        right = (mid + N//4) % N
        span  = int(self.win_rad / msg.angle_increment)

        def md(idx):
            window = np.concatenate([r[idx-span:idx], r[idx:idx+span+1]])
            w = window[(window>0)&~np.isinf(window)]
            return w.min() if w.size else np.nan

        # 실제 전방은 배열상의 back 인덱스에서
        self.d_f = md(back)
        self.d_b = md(mid)
        self.d_l = md(right)
        self.d_r = md(left)

        self.scan_ranges = r

    # ── 제어 루프 ──
    def control_loop(self):
        now = time.time()
        dt  = now - self.prev_time
        self.prev_time = now

        # 비전(포즈·전방·목표) 준비 확인
        if None in (self.wx_r, self.wy_r, self.wx_f, self.wy_f) or self.target is None:
            self.cmd_pub.publish(Twist())
            return

        # 목표 벡터·거리·오차 계산
        tx, ty = self.target
        fv = (self.wx_f - self.wx_r, self.wy_f - self.wy_r)
        tv = (tx - self.wx_r,        ty - self.wy_r)
        dist = math.hypot(*tv)
        err  = math.degrees(math.atan2(
            fv[0]*tv[1] - fv[1]*tv[0],
            fv[0]*tv[0] + fv[1]*tv[1]
        ))

        # 목표 도달 체크
        if dist <= self.dist_tolerance:
            self.get_logger().info("■ arrived at target — stopping motion")
            self.target = None
            self.state  = 'IDLE'
            self.cmd_pub.publish(Twist())
            return

        # 전방 장애물 정지 검사
        if self.scan_ranges is not None and not math.isnan(self.d_f) and self.d_f < self.obs_threshold:
            self.get_logger().info(f"▷ obstacle within {self.obs_threshold*100:.0f} cm — pausing")
            self.cmd_pub.publish(Twist())
            return

        # 원본 상태 머신
        twist = Twist()
        if self.state == 'IDLE':
            if abs(err) >= self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0, err)
                self.accum_time   = 0.0
                self.state        = 'ROTATING'
            else:
                self.state = 'DRIVING'

        elif self.state == 'ROTATING':
            self.accum_time += dt
            if self.accum_time < self.turn_interval:
                twist.angular.z = self.rotation_dir * self.fixed_omega
            else:
                self.accum_time = 0.0
                self.state      = 'PAUSE'

        elif self.state == 'PAUSE':
            self.accum_time += dt
            if self.accum_time >= self.pause_interval:
                self.accum_time = 0.0
                if abs(err) >= self.angle_tolerance:
                    self.rotation_dir = math.copysign(1.0, err)
                    self.state        = 'ROTATING'
                else:
                    self.state        = 'DRIVING'

        elif self.state == 'DRIVING':
            if abs(err) >= self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0, err)
                self.accum_time   = 0.0
                self.state        = 'ROTATING'
                twist.angular.z   = self.rotation_dir * self.fixed_omega
            else:
                twist.linear.x = self.drive_speed

        # 명령 발행
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
