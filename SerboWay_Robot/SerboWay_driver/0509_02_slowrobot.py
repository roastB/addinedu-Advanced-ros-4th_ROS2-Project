#!/usr/bin/env python3
import math, rclpy
from rclpy.node        import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg      import Int32

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        # 구독: robot_pose, robot_front, waypoint_6/7/8, target_marker
        self.create_subscription(PointStamped, '/robot_pose',    self.pose_cb,   10)
        self.create_subscription(PointStamped, '/robot_front',   self.front_cb,  10)
        self.marker_world = {}
        for wid in (6,7,8):
            self.create_subscription(
                PointStamped,
                f'/waypoint_{wid}',
                lambda msg, wid=wid: self.wp_cb(wid, msg),
                10
            )
        self.create_subscription(Int32, '/target_marker', self.target_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.target = None
        self.running = False

        self.create_timer(1/30.0, self.control_loop)

    def pose_cb(self, msg):
        self.wx_r, self.wy_r = msg.point.x, msg.point.y

    def front_cb(self, msg):
        self.wx_f, self.wy_f = msg.point.x, msg.point.y

    def wp_cb(self, wid, msg):
        self.marker_world[wid] = (msg.point.x, msg.point.y)

    def target_cb(self, msg):
        tid = msg.data
        # 목표점 좌표가 들어와 있고 로봇 위치 알고 있으면
        if tid in self.marker_world and self.wx_r is not None:
            self.target = self.marker_world[tid]
            self.running = True

    def control_loop(self):
        if not self.running or None in (self.wx_r, self.wy_r, self.wx_f, self.wy_f, self.target):
            return

        tx, ty = self.target
        fv = (self.wx_f - self.wx_r, self.wy_f - self.wy_r)
        tv = (tx - self.wx_r,        ty - self.wy_r)

        dist = math.hypot(*tv)
        err  = math.degrees(math.atan2(
            fv[0]*tv[1] - fv[1]*tv[0],
            fv[0]*tv[0] + fv[1]*tv[1]
        ))

        # 제어 상수 (각속도 극단적으로 감소)
        K_ang   = 0.005 * 0.01   # 원래 0.005의 1/100
        K_lin   = 0.002 * 0.1    # 선속도 이득 1/10
        MAX_ang = 0.3   * 0.01   # 원래 0.3의 1/100
        MAX_lin = 0.2   * 0.1    # 최대 선속도 1/10

        twist = Twist()

        # 회전 ±2° 이내에는 직진, 이상일 때 제자리 회전
        if abs(err) >= 2.0:
            twist.angular.z = max(-MAX_ang, min(MAX_ang, K_ang * err))
            twist.linear.x  = 0.0
        else:
            twist.angular.z = 0.0
            sf = max(0.0, math.cos(math.radians(err)))
            twist.linear.x = max(0.0, min(MAX_lin, K_lin * dist * sf))
            # 목표 도달 시 멈춤
            if dist < 2.0:
                twist = Twist()
                self.running = False

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
