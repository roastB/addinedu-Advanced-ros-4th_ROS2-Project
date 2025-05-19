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
        # 토픽 구독·퍼블리셔 설정
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
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 상태 변수
        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.marker_world = {}
        self.target = None
        self.d_f = self.d_l = self.d_r = float('nan')

        # 제어 파라미터
        self.fixed_omega       = 0.5   # 회전 속도 (rad/s)
        self.drive_speed       = 0.1   # 전진 속도 (m/s)
        self.angle_tolerance   = 5.0   # 목표 방향 허용치 (deg)
        self.dist_tolerance    = 1.0   # 목표 도달 기준 (m)
        self.obs_detect_thresh = 0.20  # 장애물 감지 기준 (m)
        self.obs_clear_thresh  = 0.25  # 장애물 해제 기준 (m)

        # 회전/멈춤 분할 파라미터
        self.turn_interval  = 0.05   # 회전 제어 주기 (s)
        self.pause_interval = 0.07   # 멈춤 제어 주기 (s)
        self.accum_time     = 0.0    # 누적 시간

        # 추가 전진 시간
        self.extra_drive_time = 1.15  # 장애물 회피 후 전진 시간 (s)

        # 상태 머신 초기화
        self.state         = 'IDLE'
        self.rotation_dir  = 0.0
        self._obs_heading0 = 0.0
        self.prev_time     = time.time()

        # 제어 루프 (200Hz)
        self.create_timer(1/200.0, self.control_loop)

    # 콜백들
    def pose_cb(self, msg: PointStamped):
        self.wx_r, self.wy_r = msg.point.x, msg.point.y

    def front_cb(self, msg: PointStamped):
        self.wx_f, self.wy_f = msg.point.x, msg.point.y

    def wp_cb(self, wid, msg: PointStamped):
        self.marker_world[wid] = (msg.point.x, msg.point.y)

    def target_cb(self, msg: Int32):
        tid = msg.data
        if tid in self.marker_world and self.wx_r is not None:
            self.target     = self.marker_world[tid]
            self.state      = 'ROTATING'
            self.accum_time = 0.0
            # 이전 회피 관련 플래그 삭제
            for attr in ('drive_obs_cleared','surf_obs_detected','surf_clear_time',
                         'rotobs_check_start','rotsurf_check_start'):
                if hasattr(self, attr):
                    delattr(self, attr)

    def scan_cb(self, msg: LaserScan):
        r = np.array(msg.ranges)
        N = len(r)
        mid       = N // 2
        front_idx = (mid + N//2) % N
        left_idx  = (mid - N//4) % N
        right_idx = (mid + N//4) % N
        span = int(math.radians(5.0) / msg.angle_increment)
        def md(idx):
            w = np.concatenate([r[idx-span:idx], r[idx:idx+span+1]])
            v = w[(w > 0) & ~np.isinf(w)]
            return float(v.min()) if v.size else float('nan')
        self.d_f = md(front_idx)
        self.d_l = md(left_idx)
        self.d_r = md(right_idx)

    # 제어 루프
    def control_loop(self):
        now = time.time()
        dt  = now - self.prev_time
        self.prev_time = now
        twist = Twist()

        # 준비 상태: pose 혹은 target 미수신 시 멈춤
        if self.target is None or None in (self.wx_r, self.wx_f):
            self.cmd_pub.publish(twist)
            return

        # 1) 첫 회피 전진 (DRIVE_OBS)
        if self.state == 'DRIVE_OBS':
            if not hasattr(self, 'drive_obs_cleared'):
                if not math.isnan(self.d_f) and self.d_f >= self.obs_detect_thresh:
                    self.drive_obs_cleared = now
                twist.linear.x = self.drive_speed
            elif now - self.drive_obs_cleared >= self.extra_drive_time:
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

        # 2) 첫 회피 회전 (ROTATING_OBS) — 미니 회전+멈춤 루프
        if self.state == 'ROTATING_OBS':
            heading = math.atan2(self.wy_f - self.wy_r,
                                  self.wx_f - self.wx_r)
            delta   = (heading - self._obs_heading0 + math.pi) % (2*math.pi) - math.pi

            if abs(delta) < math.pi/2 - 0.05:
                if self.accum_time < self.turn_interval:
                    twist.angular.z = self.rotation_dir * self.fixed_omega
                elif self.accum_time < self.turn_interval + self.pause_interval:
                    twist.angular.z = 0.0
                else:
                    self.accum_time = 0.0
                self.accum_time += dt
                self.cmd_pub.publish(twist)
            else:
                self.state      = 'DRIVE_OBS'
                self.accum_time = 0.0
                self.get_logger().info("▷ ROTATING_OBS done — DRIVE_OBS")
            return

        # 3) 두 번째 회피 회전 (ROTATE_SURFACE) — 미니 회전+멈춤 루프
        if self.state == 'ROTATE_SURFACE':
            heading = math.atan2(self.wy_f - self.wy_r,
                                  self.wx_f - self.wx_r)
            delta   = (heading - self._obs_heading0 + math.pi) % (2*math.pi) - math.pi

            if abs(delta) < math.pi/2 - 0.05:
                if self.accum_time < self.turn_interval:
                    twist.angular.z = self.rotation_dir * self.fixed_omega
                elif self.accum_time < self.turn_interval + self.pause_interval:
                    twist.angular.z = 0.0
                else:
                    self.accum_time = 0.0
                self.accum_time += dt
                self.cmd_pub.publish(twist)
            else:
                self.state      = 'SURFACE_DRIVE'
                self.accum_time = 0.0
                self.get_logger().info("▷ ROTATE_SURFACE done — SURFACE_DRIVE")
            return

        # 4) 표면 주행 전진 (SURFACE_DRIVE) with debug logs
        if self.state == 'SURFACE_DRIVE':
            side = self.d_l if self.rotation_dir > 0 else self.d_r
            if not hasattr(self, 'surf_obs_detected'):
                self.get_logger().info(f"[SURF] Waiting for obstacle — side={side:.2f} m")
                if not math.isnan(side) and side <= self.obs_detect_thresh:
                    self.surf_obs_detected = now
                    self.get_logger().info("[SURF] Obstacle detected")
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return

            if not hasattr(self, 'surf_clear_time'):
                self.get_logger().info(f"[SURF] Waiting for clearance — side={side:.2f} m")
                if not math.isnan(side) and side >= self.obs_clear_thresh:
                    self.surf_clear_time = now
                    self.get_logger().info("[SURF] Clearance detected")
                twist.linear.x = self.drive_speed
                self.cmd_pub.publish(twist)
                return

            elapsed = now - self.surf_clear_time
            self.get_logger().info(f"[SURF] Extra driving — elapsed={elapsed:.2f}s")
            if elapsed >= self.extra_drive_time:
                self.get_logger().info("▷ SURFACE_DRIVE done — resume to target")
                self.state = 'ROTATING'
                for attr in ('surf_obs_detected','surf_clear_time'):
                    if hasattr(self, attr):
                        delattr(self, attr)
                self.cmd_pub.publish(Twist())
                return

            twist.linear.x = self.drive_speed
            self.cmd_pub.publish(twist)
            return

        # --- 목표 주행 상태 머신 (IDLE→ROTATING→CONFIRM_OBS→STOP_OBS→...) ---
        tx, ty = self.target
        fv = (self.wx_f - self.wx_r, self.wy_f - self.wy_r)
        tv = (tx - self.wx_r,         ty - self.wy_r)
        dist = math.hypot(*tv)
        err  = math.degrees(
            math.atan2(fv[0]*tv[1] - fv[1]*tv[0],
                       fv[0]*tv[0] + fv[1]*tv[1])
        )

        # 목표 도착
        if dist <= self.dist_tolerance:
            self.get_logger().info("■ arrived — stopping motion")
            self.target = None
            self.state  = 'IDLE'
            self.cmd_pub.publish(Twist())
            return

        # DRIVING → CONFIRM_OBS
        if self.state == 'DRIVING' and not math.isnan(self.d_f) and self.d_f < self.obs_detect_thresh:
            self.state      = 'CONFIRM_OBS'
            self.accum_time = 0.0
            self.get_logger().info("▷ possible obstacle — CONFIRM_OBS")
            return

        # CONFIRM_OBS: 1초간 장애물 유지 확인 (멈춤)
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
            # 대기할 땐 무조건 멈춤
            self.cmd_pub.publish(Twist())
            return

        # STOP_OBS → ROTATING_OBS
        if self.state == 'STOP_OBS':
            self.accum_time += dt
            if self.accum_time >= self.pause_interval:
                self._obs_heading0 = math.atan2(self.wy_f - self.wy_r,
                                                self.wx_f - self.wx_r)
                self.rotation_dir  = +1.0 if self.d_l > self.d_r else -1.0
                self.state         = 'ROTATING_OBS'
                self.accum_time    = 0.0
                self.get_logger().info("▷ obstacle — start ROTATING_OBS")
            self.cmd_pub.publish(Twist())
            return

        # IDLE → ROTATING/DRIVING
        if self.state == 'IDLE':
            if abs(err) >= self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0, err)
                self.state        = 'ROTATING'
                self.accum_time   = 0.0
            else:
                self.state        = 'DRIVING'

        if self.state == 'ROTATING':
            self.accum_time += dt
            if self.accum_time < self.turn_interval:
                twist.angular.z = self.rotation_dir * self.fixed_omega
            else:
                self.state      = 'PAUSE'
                self.accum_time = 0.0

        elif self.state == 'PAUSE':
            self.accum_time += dt
            if self.accum_time >= self.pause_interval:
                self.state        = 'ROTATING' if abs(err) >= self.angle_tolerance else 'DRIVING'
                self.rotation_dir = math.copysign(1.0, err)
                self.accum_time   = 0.0

        elif self.state == 'DRIVING':
            twist.linear.x = self.drive_speed

        self.cmd_pub.publish(twist)

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
