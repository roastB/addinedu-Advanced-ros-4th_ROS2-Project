#!/usr/bin/env python3
import math
import pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg      import Int32, Empty

# Calibration and map settings
CALIB_PATH     = '/home/addinedu/camera_calibration.pkl'
MAP_WIDTH_CM   = 200.0
MAP_HEIGHT_CM  = 100.0
MARKER_SIZE_CM = 10.0
half = MARKER_SIZE_CM / 2.0

# World marker coordinates (cm)
marker_world = {
    0: (0 + half,                  0 + half),
    1: (MAP_WIDTH_CM - half,       0 + half),
    3: (0 + half,                  MAP_HEIGHT_CM - half),
    4: (MAP_WIDTH_CM - half,       MAP_HEIGHT_CM - half),
}

class ArucoNavCommander(Node):
    def __init__(self):
        super().__init__('aruco_nav_commander')

        # Publishers
        self.pose_pub    = self.create_publisher(PointStamped, '/robot_pose',  10)
        self.front_pub   = self.create_publisher(PointStamped, '/robot_front', 10)
        self.target_pub  = self.create_publisher(PointStamped, '/target_point', 10)
        self.arrived_pub = self.create_publisher(Empty,         '/arrived',      10)

        # Load camera calibration
        with open(CALIB_PATH, 'rb') as f:
            calib = pickle.load(f)
        self.mtx, self.dist = calib['camera_matrix'], calib['dist_coeffs']
        self.aruco_dict     = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params         = aruco.DetectorParameters()
        self.marker_length  = MARKER_SIZE_CM
        self.cap            = cv2.VideoCapture(2, cv2.CAP_V4L2)

        self.cached_H      = None
        self.current_pose  = None
        self.active_target = None

        # ID to world coords (for non-vertex markers)
        self.id_to_coord = {
            11: (44, 31),    # P1
            12: (138, 31),   # P2
            13: (144, 67),   # P3
            14: (42, 71),    # P4
            21: (70, 67),    # T1
            22: (94, 67),    # T2
            23: (118, 67),   # T3
            31: (144, 29),   # PICK
            32: (32, 67),    # DROP
            98: (92, 31),    # mid
            99: (92, 15),    # HOME
        }

        # Predefined paths (marker ID sequences)
        self.paths = {
            1: [12, 13, 21],
            2: [12, 13, 22],
            3: [12, 13, 23],
            4: [98, 12, 13, 21, 32],
            5: [98, 12, 13, 22, 32],
            6: [98, 12, 13, 23, 32],
            0: [14, 11, 98, 99],
            9: [98, 12, 31],
        }

        self.path_queue   = []

        # Windows for visualization
        cv2.namedWindow("win", cv2.WINDOW_NORMAL)
        cv2.namedWindow("tf_map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("win", 640, 480)
        cv2.resizeWindow("tf_map", int(MAP_WIDTH_CM*5*1.25), int(MAP_HEIGHT_CM*5*1.25))

        # Timers
        self.create_timer(1/30.0, self.vision_cb)
        self.create_timer(0.5,    self.check_and_publish_next)

        self.get_logger().info("경로 번호 (키보드 0~9) 입력 후 Enter 없이 → 좌표 순차 발행")

    def build_homography(self, corners, ids):
        if ids is None:
            return None
        img_pts, world_pts = [], []
        for i, mid in enumerate(ids.flatten()):
            if mid in marker_world:
                pts = corners[i][0]
                img_pts.append([float(pts[:,0].mean()), float(pts[:,1].mean())])
                world_pts.append(marker_world[mid])
        if len(img_pts) < 4:
            return None
        H, _ = cv2.findHomography(np.array(img_pts, np.float32), np.array(world_pts, np.float32))
        return H

    def pixel_to_world(self, H, px, py):
        pt = np.array([px, py, 1.0], dtype=np.float32)
        w  = H @ pt; w /= w[2]
        return float(w[0]), float(w[1])

    def vision_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            cv2.waitKey(1)
            return

        gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        aruco.drawDetectedMarkers(frame, corners, ids)

        H_new = self.build_homography(corners, ids)
        if H_new is not None:
            self.cached_H = H_new

        if ids is not None and self.cached_H is not None and 5 in ids.flatten():
            i5 = list(ids.flatten()).index(5)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.mtx, self.dist)
            R, _ = cv2.Rodrigues(rvecs[i5][0])
            c5 = corners[i5][0]
            px, py = float(c5[:,0].mean()), float(c5[:,1].mean())
            wx_r, wy_r = self.pixel_to_world(self.cached_H, px, py)
            front3d = np.array([[0.0, self.marker_length, 0.0]], dtype=np.float32)
            camf = (R @ front3d.T + tvecs[i5][0].reshape(3,1)).T
            imgpts, _ = cv2.projectPoints(camf, np.zeros(3), np.zeros(3), self.mtx, self.dist)
            fx, fy = imgpts[0].ravel().astype(int)
            wx_f, wy_f = self.pixel_to_world(self.cached_H, fx, fy)

            for pub, x, y in [(self.pose_pub, wx_r, wy_r), (self.front_pub, wx_f, wy_f)]:
                msg = PointStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x = x
                msg.point.y = y
                msg.point.z = 0.0
                pub.publish(msg)

            if self.active_target:
                dx, dy = self.active_target[0] - wx_r, self.active_target[1] - wy_r
                if math.hypot(dx, dy) <= 2.0:
                    self.get_logger().info("■ 도착 — next 좌표 발행 및 arrived 신호")
                    self.arrived_pub.publish(Empty())
                    self.active_target = None

        cv2.imshow("win", frame)
        if self.cached_H is not None:
            PPM = 5
            MAP_W = int(MAP_WIDTH_CM * PPM)
            MAP_H = int(MAP_HEIGHT_CM * PPM)
            S = np.array([[PPM,0,0],[0,-PPM,MAP_H],[0,0,1]], np.float32)
            tf_map = cv2.warpPerspective(frame, S @ self.cached_H, (MAP_W, MAP_H))
            for (x, y) in self.path_queue:
                x_px, y_px = int(x*PPM), int(MAP_H - y*PPM)
                cv2.circle(tf_map, (x_px, y_px), 6, (0,255,0), -1)
            if self.active_target:
                x, y = self.active_target
                x_px, y_px = int(x*PPM), int(MAP_H - y*PPM)
                cv2.circle(tf_map, (x_px, y_px), 8, (0,0,255), -1)
            cv2.imshow("tf_map", tf_map)

        key = cv2.waitKey(1) & 0xFF
        if chr(key).isdigit():
            pid = int(chr(key))
            if pid in self.paths:
                self.path_queue = [self.id_to_coord[i] for i in self.paths[pid]]
                self.active_target = None
                self.get_logger().info(f"✅ 경로 {pid} 시작 — 총 {len(self.path_queue)}개 좌표")
        elif key == ord('q'):
            rclpy.shutdown()

    def check_and_publish_next(self):
        if self.active_target or not self.path_queue:
            return
        x, y = self.path_queue.pop(0)
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.target_pub.publish(msg)
        self.active_target = (x, y)
        self.get_logger().info(f"📍 좌표 전송: ({x:.1f}, {y:.1f})")


def main():
    rclpy.init()
    node = ArucoNavCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()