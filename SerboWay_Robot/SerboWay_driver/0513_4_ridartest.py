#!/usr/bin/env python3, 방향 맞추기
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

        # LiDAR 구독
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)

        # Matplotlib 준비
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        # 맵 전체 영역으로 축 고정
        self.ax.set_xlim(-self.map_w/2, self.map_w/2)
        self.ax.set_ylim(-self.map_h/2, self.map_h/2)
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlabel('X (m) ← left | right →')
        self.ax.set_ylabel('Y (m) ↓ back | front ↑')
        self.ax.set_title('Full LiDAR Map (flipped LR & UD)')

        # 아주 작은 파란 점
        self.scatter = self.ax.scatter([], [], s=1, c='blue')

        # 좌/우 반전
        self.ax.invert_xaxis()
        # 상/하 반전
        self.ax.invert_yaxis()

        # 창 최대화 (Qt 백엔드)
        try:
            mng = plt.get_current_fig_manager()
            mng.window.showMaximized()
        except Exception:
            pass

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def cb(self, msg: LaserScan):
        # 1) 거리→로봇 좌표계(x=front, y=left)
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)
        valid = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])

        # 2) 로봇→화면 매핑 (전방↑, 좌측←)
        rot_x = -ys
        rot_y =  xs

        # 3) 화면 범위 필터
        mask = (rot_x >= -self.map_w/2) & (rot_x <= self.map_w/2) & \
               (rot_y >= -self.map_h/2) & (rot_y <= self.map_h/2)
        rot_x, rot_y = rot_x[mask], rot_y[mask]

        # 4) 그리기
        self.scatter.set_offsets(np.c_[rot_x, rot_y])
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
