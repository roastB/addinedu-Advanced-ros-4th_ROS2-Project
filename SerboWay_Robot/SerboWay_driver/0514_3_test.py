#!/usr/bin/env python3
import math, os, pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node         import Node
from geometry_msgs.msg   import PointStamped
from std_msgs.msg        import Int32

# ===== 설정 =====
CALIB_PATH   = '/home/addinedu/camera_calibration.pkl'
MAP_WIDTH_CM  = 100.0
MAP_HEIGHT_CM = 100.0
MARKER_SIZE_CM = 10.0
half = MARKER_SIZE_CM / 2.0

# 호모그래피 기준 마커 월드 좌표 (cm)
marker_world = {
    0: (0+half,               0+half),
    1: (MAP_WIDTH_CM-half,    0+half),
    3: (0+half,               MAP_HEIGHT_CM-half),
    4: (MAP_WIDTH_CM-half,    MAP_HEIGHT_CM-half),
}

WAYPOINT_IDS = [6,7,8]
PPM   = 5
MAP_W = int(MAP_WIDTH_CM  * PPM)
MAP_H = int(MAP_HEIGHT_CM * PPM)

class ArucoNav(Node):
    def __init__(self):
        super().__init__('aruco_nav')
        # 퍼블리셔 설정
        self.pose_pub    = self.create_publisher(PointStamped, '/robot_pose',    10)
        self.front_pub   = self.create_publisher(PointStamped, '/robot_front',   10)
        self.target_pub  = self.create_publisher(Int32,        '/target_marker', 10)
        self.wp_pubs     = {
            wid: self.create_publisher(PointStamped, f'/waypoint_{wid}', 10)
            for wid in WAYPOINT_IDS
        }
        self.wp_sent     = set()
        self.wp_coords   = {}
        self.last_target = None
        self.cached_H    = None

        # 카메라 캘리브레이션 로드
        with open(CALIB_PATH, 'rb') as f:
            calib = pickle.load(f)
        self.mtx, self.dist = calib['camera_matrix'], calib['dist_coeffs']

        # ArUco 설정
        self.aruco_dict    = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params        = aruco.DetectorParameters()
        self.marker_length = MARKER_SIZE_CM

        # OpenCV 창
        cv2.namedWindow("win",    cv2.WINDOW_NORMAL)
        cv2.resizeWindow("win",    640, 480)
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

        # 타이머 설정 (30Hz)
        self.create_timer(1/30.0, self.cb)

    def build_homography(self, corners, ids):
        if ids is None:
            return None
        img_pts, world_pts = [], []
        for i, mid in enumerate(ids.flatten()):
            if mid in marker_world:
                c = corners[i][0]
                img_pts.append([float(c[:,0].mean()), float(c[:,1].mean())])
                world_pts.append(marker_world[mid])
        if len(img_pts) < 4:
            return None
        H, _ = cv2.findHomography(
            np.array(img_pts, np.float32),
            np.array(world_pts, np.float32))
        return H

    def pixel_to_world(self, H, px, py):
        pt = np.array([px, py, 1.0], dtype=np.float32)
        w  = H @ pt; w /= w[2]
        return float(w[0]), float(w[1])

    def cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Undistort → Crop → Resize
        h, w = frame.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        und = cv2.undistort(frame, self.mtx, self.dist, None, new_mtx)
        x,y,w0,h0 = roi
        if all(v>0 for v in (x,y,w0,h0)):
            und = und[y:y+h0, x:x+w0]
        img = cv2.resize(und, (640,480))

        # Detect & draw
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        aruco.drawDetectedMarkers(img, corners, ids)

        # Update homography
        H_new = self.build_homography(corners, ids)
        if H_new is not None:
            self.cached_H = H_new

        # 화면에 각 마커의 Homography 좌표 표시
        if self.cached_H is not None and ids is not None:
            y0 = 20
            for mid in (0,1,3,4,5,6,7,8):
                if mid in ids.flatten():
                    i = list(ids.flatten()).index(mid)
                    c = corners[i][0]
                    px, py = float(c[:,0].mean()), float(c[:,1].mean())
                    wx, wy = self.pixel_to_world(self.cached_H, px, py)
                    text = f"ID{mid}: ({wx:.1f},{wy:.1f})cm"
                    cv2.putText(img, text, (10, y0),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
                    y0 += 20

        # Publish robot pose as before...
        # (기존 5번 마커 Publish 로직 유지)

        cv2.imshow("win", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ArucoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
