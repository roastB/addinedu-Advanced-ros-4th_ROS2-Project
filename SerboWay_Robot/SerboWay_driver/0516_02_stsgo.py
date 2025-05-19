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
                PointStamped, f'/waypoint_{wid}',
                lambda msg, wid=wid: self.wp_cb(wid, msg),
                10
            )
        self.create_subscription(Int32, '/target_marker', self.target_cb, 10)
        # 2) LiDAR 구독
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        # 3) cmd_vel 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 상태 변수
        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.marker_world = {}
        self.target = None
        self.d_f = self.d_l = self.d_r = float('nan')

        # 제어 파라미터
        self.fixed_omega     = 0.5    # rad/s 회전 속도
        self.drive_speed     = 0.1    # m/s 전진 속도
        self.angle_tolerance = 5.0    # deg 헤딩 허용치
        self.dist_tolerance  = 1.0    # m 목표 도달 기준
        self.obs_threshold   = 0.2    # m 장애물 감지 기준

        # 끊어서 회전용 파라미터
        self.turn_interval   = 0.05   # s 회전 시간
        self.pause_interval  = 0.07   # s 멈춤 시간
        self.accum_time      = 0.0

        # 장애물 회피 후 추가 전진 시간 (s)
        self.extra_drive_time   = 1.5
        self.drive_obs_start    = None

        # 상태 머신 변수
        # IDLE, ROTATING, PAUSE, DRIVING,
        # STOP_OBS, ROTATING_OBS, PAUSE_OBS,
        # DRIVE_OBS, FINISHED
        self.state        = 'IDLE'
        self.rotation_dir = 0.0
        self._obs_heading0 = 0.0
        self.prev_time     = time.time()

        # 제어 루프 타이머 (200Hz)
        self.create_timer(1/200.0, self.control_loop)

    # ── 콜백들 ──
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
            self.target      = self.marker_world[tid]
            self.state       = 'IDLE'
            self.accum_time  = 0.0

    def scan_cb(self, msg: LaserScan):
        r = np.array(msg.ranges)
        N = len(r)
        mid        = N // 2
        front_idx  = (mid + N//2) % N
        left_idx   = (mid - N//4) % N
        right_idx  = (mid + N//4) % N
        span = int(math.radians(5.0) / msg.angle_increment)
        def md(idx):
            w = np.concatenate([r[idx-span:idx], r[idx:idx+span+1]])
            v = w[(w > 0) & ~np.isinf(w)]
            return float(v.min()) if v.size else float('nan')
        self.d_f = md(front_idx)
        self.d_l = md(left_idx)
        self.d_r = md(right_idx)

    # ── 메인 제어 루프 ──
    def control_loop(self):
        now = time.time()
        dt  = now - self.prev_time
        self.prev_time = now
        twist = Twist()

        # 준비 여부
        if None in (self.wx_r, self.wx_f) or self.target is None:
            self.cmd_pub.publish(twist)
            return

        # 장애물 회피 후 추가 전진 상태
        if self.state == 'DRIVE_OBS':
            # 충분한 시간(0.5s) 경과했으면 사이드 거리 체크
            if now - self.drive_obs_start >= self.extra_drive_time:
                side_dist = self.d_r if self.rotation_dir > 0 else self.d_l
                if not math.isnan(side_dist) and side_dist >= 0.25:
                    self.get_logger().info("▷ obstacle cleared — resume normal")
                    self.state = 'IDLE'
                    self.cmd_pub.publish(Twist())
                    return
            # 아직 최소 시간 미만이거나 거리 미달 시 계속 전진
            twist.linear.x = self.drive_speed
            self.cmd_pub.publish(twist)
            return

        # 목표 계산
        tx, ty = self.target
        fv = (self.wx_f - self.wx_r, self.wy_f - self.wy_r)
        tv = (tx - self.wx_r,        ty - self.wy_r)
        dist = math.hypot(*tv)
        err  = math.degrees(math.atan2(
            fv[0]*tv[1] - fv[1]*tv[0],
            fv[0]*tv[0] + fv[1]*tv[1]
        ))

        # 목표 도착
        if dist <= self.dist_tolerance:
            self.get_logger().info("■ arrived — stopping motion")
            self.target = None
            self.state  = 'IDLE'
            self.cmd_pub.publish(Twist())
            return

        # DRIVING 중 장애물 감지 시 STOP_OBS
        if self.state == 'DRIVING' and not math.isnan(self.d_f) and self.d_f < self.obs_threshold:
            self.state       = 'STOP_OBS'
            self.accum_time  = 0.0
            self.get_logger().info("▷ hit obstacle — STOP_OBS")
            return

        # STOP_OBS: 멈추고 대기 후 ROTATING_OBS
        if self.state == 'STOP_OBS':
            self.accum_time += dt
            self.cmd_pub.publish(Twist())
            if self.accum_time >= self.pause_interval:
                self._obs_heading0   = math.atan2(self.wy_f - self.wy_r, self.wx_f - self.wx_r)
                self.rotation_dir    = +1.0 if self.d_l > self.d_r else -1.0
                self.accum_time      = 0.0
                self.state           = 'ROTATING_OBS'
                self.get_logger().info("▷ obstacle — start ROTATING_OBS")
            return

        # ROTATING_OBS: 끊어서 회전하며 90°까지
        if self.state == 'ROTATING_OBS':
            self.accum_time += dt
            heading = math.atan2(self.wy_f - self.wy_r, self.wx_f - self.wx_r)
            delta   = self._angle_diff(heading, self._obs_heading0)
            if self.accum_time < self.turn_interval:
                twist.angular.z = self.rotation_dir * self.fixed_omega
                self.cmd_pub.publish(twist)
            else:
                self.accum_time = 0.0
                # 90° 회전 완료 시 → DRIVE_OBS로 전환
                if abs(delta) >= math.pi/2:
                    self.state            = 'DRIVE_OBS'
                    self.drive_obs_start  = now
                    self.get_logger().info("▷ 90° ROTATING_OBS done — DRIVE_OBS")
                    self.cmd_pub.publish(Twist())  # 정지 후 바로 전진 시작
                else:
                    self.state = 'PAUSE_OBS'
                return

        # PAUSE_OBS: mini-pause 후 다시 ROTATING_OBS (정지 명령 포함)
        if self.state == 'PAUSE_OBS':
            self.cmd_pub.publish(Twist())
            self.accum_time += dt
            if self.accum_time >= self.pause_interval:
                self.accum_time = 0.0
                self.state      = 'ROTATING_OBS'
            return

        # ───────── 일반 주행 상태 머신 ─────────
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
            else:
                twist.linear.x = self.drive_speed

        # 모든 경우 twist 퍼블리시
        self.cmd_pub.publish(twist)

    # 두 각 차이를 -π~π로 정규화
    def _angle_diff(self, a, b):
        d = a - b
        return (d + math.pi) % (2*math.pi) - math.pi

def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
