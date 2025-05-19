#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg      import Int32

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        # 구독
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
        # cmd_vel 발행
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 상태 변수
        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.marker_world = {}
        self.target = None

        # 제어 파라미터
        self.fixed_omega     = 0.1    # rad/s 회전 속도
        self.turn_interval   = 0.05   # s 회전 시간
        self.pause_interval  = 0.05    # s 대기 시간
        self.drive_speed     = 0.1    # m/s 전진 속도
        self.angle_tolerance = 10    # deg
        self.dist_tolerance  = 0.05   # m

        # 상태 머신
        self.state        = 'IDLE'
        self.rotation_dir = 0.0
        self.accum_time   = 0.0
        self.prev_time    = time.time()

        # 고속 타이머 (5ms 단위)
        self.create_timer(1/200.0, self.control_loop)

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

    def control_loop(self):
        now = time.time()
        dt  = now - self.prev_time
        self.prev_time = now

        # 준비 안됐거나 목표 없으면 완전 정지
        if None in (self.wx_r, self.wy_r, self.wx_f, self.wy_f) or self.target is None:
            self.cmd_pub.publish(Twist())
            return

        tx, ty = self.target
        fv = (self.wx_f - self.wx_r, self.wy_f - self.wy_r)
        tv = (tx - self.wx_r,        ty - self.wy_r)
        dist = math.hypot(*tv)
        err  = math.degrees(math.atan2(
            fv[0]*tv[1] - fv[1]*tv[0],
            fv[0]*tv[0] + fv[1]*tv[1]
        ))

        # --- 거리 도착 우선 처리 ---
        if dist <= self.dist_tolerance:
            self.get_logger().info("■ arrived at target — stopping all motion")
            self.target = None
            self.cmd_pub.publish(Twist())
            return

        twist = Twist()

        # --- 거리 미도달: 정상 주행 상태머신 ---
        if self.state == 'IDLE':
            if abs(err) >= self.angle_tolerance:
                self.rotation_dir = math.copysign(1.0, err)
                self.accum_time   = 0.0
                self.state        = 'ROTATING'
                self.get_logger().info(f"→ ROTATING dir={self.rotation_dir:+.0f}")
            else:
                self.state = 'DRIVING'
                self.get_logger().info("→ DRIVING")

        elif self.state == 'ROTATING':
            self.accum_time += dt
            if self.accum_time < self.turn_interval:
                twist.angular.z = self.rotation_dir * self.fixed_omega
            else:
                self.accum_time = 0.0
                self.state      = 'PAUSE'
                self.get_logger().info("→ PAUSE")

        elif self.state == 'PAUSE':
            self.accum_time += dt
            if self.accum_time >= self.pause_interval:
                self.accum_time = 0.0
                if abs(err) >= self.angle_tolerance:
                    self.rotation_dir = math.copysign(1.0, err)
                    self.state        = 'ROTATING'
                    self.get_logger().info(f"↺ ROTATING again dir={self.rotation_dir:+.0f}")
                else:
                    self.state = 'DRIVING'
                    self.get_logger().info("→ DRIVING")

        elif self.state == 'DRIVING':
            # 전진 중에도 오차 체크
            if abs(err) >= self.angle_tolerance:
                self.get_logger().info("↺ DRIVING→ROTATING (angle error)")
                self.rotation_dir = math.copysign(1.0, err)
                self.state        = 'ROTATING'
                self.accum_time   = 0.0
                twist.angular.z   = self.rotation_dir * self.fixed_omega
            else:
                twist.linear.x = self.drive_speed

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
