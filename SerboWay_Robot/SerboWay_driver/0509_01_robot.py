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
        self.route = []
        self.idx   = 0
        self.running = False

        self.create_timer(1/30.0, self.control_loop)

    def pose_cb(self, msg):
        self.wx_r, self.wy_r = msg.point.x, msg.point.y

    def front_cb(self, msg):
        self.wx_f, self.wy_f = msg.point.x, msg.point.y

    def wp_cb(self, wid, msg):
        # 동적 waypoint 좌표 저장
        self.marker_world[wid] = (msg.point.x, msg.point.y)

    def target_cb(self, msg):
        tid = msg.data
        if tid not in self.marker_world or self.wx_r is None:
            return
        tx, ty = self.marker_world[tid]
        sx, sy = self.wx_r, self.wy_r

        # 6분할 경로 생성
        self.route = []
        for k in range(1,7):
            frac = k/6.0
            mx = sx + (tx - sx)*frac
            my = sy + (ty - sy)*frac
            self.route.append((mx, my))
        self.idx     = 0
        self.running = True

    def control_loop(self):
        if not self.running or None in (self.wx_r, self.wy_r, self.wx_f, self.wy_f):
            return
        if self.idx >= len(self.route):
            self.cmd_pub.publish(Twist())
            self.running = False
            return

        tx, ty = self.route[self.idx]
        fv = (self.wx_f - self.wx_r, self.wy_f - self.wy_r)
        tv = (tx - self.wx_r,       ty - self.wy_r)

        dist = math.hypot(*tv)
        err  = math.degrees(math.atan2(
            fv[0]*tv[1] - fv[1]*tv[0],
            fv[0]*tv[0] + fv[1]*tv[1]
        ))

        # 제어 상수
        K_ang, K_lin     = 0.005, 0.002
        MAX_ang, MAX_lin = 0.3,   0.2
        MIN_f            = 0.1

        twist = Twist()
        twist.angular.z = max(-MAX_ang, min(MAX_ang, K_ang * err))
        sf = max(MIN_f, math.cos(math.radians(err)))
        twist.linear.x  = max(0.0, min(MAX_lin, K_lin * dist * sf))

        if dist < 2.0:
            self.idx += 1

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
