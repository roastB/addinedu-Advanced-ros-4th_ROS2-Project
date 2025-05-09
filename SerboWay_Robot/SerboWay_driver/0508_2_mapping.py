#!/usr/bin/env python3
import sys
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    import numpy as np
    import matplotlib.pyplot as plt
except ImportError as e:
    print(f"[ERROR] 필요한 모듈이 없습니다: {e}")
    sys.exit(1)

class FullMapViewer(Node):
    def __init__(self):
        super().__init__('full_map_viewer')
        # 맵 크기(m) 파라미터
        self.declare_parameter('map_width', 2.0)
        self.declare_parameter('map_height', 1.0)
        self.map_w = self.get_parameter('map_width').value
        self.map_h = self.get_parameter('map_height').value

        # ROS 구독
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)

        # Matplotlib 준비
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        # 맵 전체 영역으로 축 고정
        self.ax.set_xlim(-self.map_w/2, self.map_w/2)
        self.ax.set_ylim(-self.map_h/2, self.map_h/2)
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Full LiDAR Map')
        # 아주 작은 파란 점
        self.scatter = self.ax.scatter([], [], s=1, c='blue')
        # 창 최대화
        try:
            mng = plt.get_current_fig_manager()
            mng.window.showMaximized()    # Qt backend
        except Exception:
            pass
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def cb(self, msg: LaserScan):
        # 거리→XY 변환
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)
        valid = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])

        # 전체 맵 범위 안의 점만 필터
        mask = (xs >= -self.map_w/2) & (xs <= self.map_w/2) & \
               (ys >= -self.map_h/2) & (ys <= self.map_h/2)
        xs, ys = xs[mask], ys[mask]

        # scatter 업데이트
        self.scatter.set_offsets(np.c_[xs, ys])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    node = FullMapViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
