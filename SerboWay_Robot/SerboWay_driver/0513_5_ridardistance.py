#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
except ImportError as e:
    print(f"[ERROR] 필요한 모듈이 없습니다: {e}")
    sys.exit(1)

class MapWith4DirDist(Node):
    def __init__(self):
        super().__init__('map_4dir_dist')
        # 파라미터
        self.declare_parameter('map_width', 2.0)
        self.declare_parameter('map_height', 1.0)
        self.declare_parameter('window_deg', 5.0)  # 각 방향 ±window_deg
        self.map_w   = self.get_parameter('map_width').value
        self.map_h   = self.get_parameter('map_height').value
        self.win_rad = np.deg2rad(self.get_parameter('window_deg').value)

        # LiDAR 구독
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)

        # 플롯 설정
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8,8))
        self.ax.set_xlim(-self.map_w/2, self.map_w/2)
        self.ax.set_ylim(-self.map_h/2, self.map_h/2)
        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('LiDAR Map + 4-Direction Distances (flipped axes)')

        # LiDAR 포인트
        self.scatter = self.ax.scatter([], [], s=1, c='blue')
        # 로봇 위치 표시 (빨간 ×)
        self.ax.scatter([0], [0],
                        marker='x', c='red', s=100, linewidths=2,
                        label='Robot')
        self.ax.legend(loc='upper right')

        # 축 뒤집기: 좌/우, 상/하 모두 반전
        self.ax.invert_xaxis()
        self.ax.invert_yaxis()

        # 거리 텍스트 4개 (배치 고정)
        fs = 16  # font size
        pad = 0.03
        # Front는 하단 중앙, 표시 텍스트는 'Back' 거리
        self.txt_front = self.ax.text(
            0.5, pad, '',  
            ha='center', va='bottom',
            transform=self.ax.transAxes,
            color='red', fontsize=fs,
            bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=3)
        )
        # Back은 상단 중앙, 표시 텍스트는 'Front' 거리
        self.txt_back = self.ax.text(
            0.5, 1 - pad, '',  
            ha='center', va='top',
            transform=self.ax.transAxes,
            color='red', fontsize=fs,
            bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=3)
        )
        # Left는 오른쪽 중앙, 표시 텍스트는 'Right' 거리
        self.txt_left = self.ax.text(
            1 - pad, 0.5, '',  
            ha='right', va='center',
            transform=self.ax.transAxes,
            color='red', fontsize=fs,
            bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=3)
        )
        # Right는 왼쪽 중앙, 표시 텍스트는 'Left' 거리
        self.txt_right = self.ax.text(
            pad, 0.5, '',  
            ha='left', va='center',
            transform=self.ax.transAxes,
            color='red', fontsize=fs,
            bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=3)
        )

        try:
            m = plt.get_current_fig_manager()
            m.window.showMaximized()
        except:
            pass

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def get_min_dist(self, ranges, center_idx, span):
        window = np.concatenate([
            ranges[center_idx-span:center_idx],
            ranges[center_idx:center_idx+span+1]
        ])
        valid = window[(window > 0) & ~np.isinf(window)]
        return valid.min() if valid.size else np.nan

    def cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # 1) 맵 그리기
        valid = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        xs = ranges[valid] * np.cos(angles[valid])
        ys = ranges[valid] * np.sin(angles[valid])
        rot_x = -ys
        rot_y =  xs
        mask = (
            (rot_x >= -self.map_w/2) & (rot_x <= self.map_w/2) &
            (rot_y >= -self.map_h/2) & (rot_y <= self.map_h/2)
        )
        self.scatter.set_offsets(np.c_[rot_x[mask], rot_y[mask]])

        # 2) 4방향 거리 계산
        N     = len(ranges)
        mid   = N//2            # 전방
        back  = (mid + N//2) % N # 후방
        left  = (mid + N//4) % N # 좌측
        right = (mid - N//4) % N # 우측
        span  = int(self.win_rad / msg.angle_increment)

        d_f = self.get_min_dist(ranges, mid, span)*100
        d_b = self.get_min_dist(ranges, back, span)*100
        d_l = self.get_min_dist(ranges, left, span)*100
        d_r = self.get_min_dist(ranges, right, span)*100

        # 텍스트 내용 업데이트 (배치에 맞춰)
        self.txt_front.set_text(f'Back: {d_f:.1f} cm')
        self.txt_back .set_text(f'Front:  {d_b:.1f} cm')
        self.txt_left .set_text(f'Right:  {d_l:.1f} cm')
        self.txt_right.set_text(f'Left: {d_r:.1f} cm')

        # 갱신
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    node = MapWith4DirDist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
