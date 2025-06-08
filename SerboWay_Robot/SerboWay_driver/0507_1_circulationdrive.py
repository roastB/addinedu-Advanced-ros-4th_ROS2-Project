#!/usr/bin/env python3
import os
import pickle
import math
import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ===== 카메라 보정 파일 경로 =====
CALIB_PATH = '/home/addinedu/camera_calibration.pkl'

# ===== 맵 기준 마커 월드 좌표 (cm) =====
marker_world = {
    0: (0,   0),
    1: (200, 0),
    4: (200, 100),
    3: (0,   100),
}

def build_homography(corners, ids):
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
        np.array(img_pts, dtype=np.float32),
        np.array(world_pts, dtype=np.float32)

    )
    return H

def pixel_to_world(H, px, py):
    pt = np.array([px, py, 1.0], dtype=np.float32)
    w  = H @ pt
    w /= w[2]
    return float(w[0]), float(w[1])


class ArucoNavigator(Node):
    def __init__(self, calib):
        super().__init__('aruco_navigator')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # calibration
        self.mtx  = calib['camera_matrix']
        self.dist = calib['dist_coeffs']

        # aruco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params     = aruco.DetectorParameters()
        self.cached_H   = None

        # pose-estimation marker length
        self.marker_length = 10.0

        # 순환할 웨이포인트 ID 리스트 (5→6→7→5…)
        self.waypoints = [5, 6, 7]
        self.wp_idx    = 0

        # 각 웨이포인트 ID별로 좌표 저장용
        self.wp_coords = {}

        # 카메라 열기
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패')
            rclpy.shutdown()

        self.create_timer(1/30.0, self.timer_cb)

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # (1) Undistort → Crop → Resize
        h, w = frame.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(
            self.mtx, self.dist, (w, h), 1, (w, h))
        und = cv2.undistort(frame, self.mtx, self.dist, None, new_mtx)
        x, y, w0, h0 = roi
        if all(v > 0 for v in [x, y, w0, h0]):
            und = und[y:y+h0, x:x+w0]
        img = cv2.resize(und, (640, 480))

        # (2) Detect & draw map markers, update H
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.params)
        aruco.drawDetectedMarkers(img, corners, ids)
        H_new = build_homography(corners, ids)
        if H_new is not None:
            self.cached_H = H_new

        # (3) Pose estimate marker 5 & draw front arrow
        wx_r = wy_r = wx_f = wy_f = None
        if ids is not None and self.cached_H is not None and 5 in ids.flatten():
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.mtx, self.dist)
            i5 = list(ids.flatten()).index(5)
            rvec, tvec = rvecs[i5][0], tvecs[i5][0]
            R, _ = cv2.Rodrigues(rvec)

            c5 = corners[i5][0]
            px_c = float(c5[:,0].mean());  py_c = float(c5[:,1].mean())
            wx_r, wy_r = pixel_to_world(self.cached_H, px_c, py_c)

            front_3d = np.array([[0.0, self.marker_length, 0.0]], dtype=np.float32)
            cam_front = (R @ front_3d.T + tvec.reshape(3,1)).T
            imgpts, _ = cv2.projectPoints(
                cam_front, np.zeros(3), np.zeros(3),
                self.mtx, self.dist)
            fx, fy = imgpts[0].ravel().astype(int)

            cv2.arrowedLine(img,
                            (int(px_c), int(py_c)),
                            (fx, fy),
                            (0,255,0), 2, tipLength=0.2)

            wx_f, wy_f = pixel_to_world(self.cached_H, fx, fy)
            cv2.putText(img, f"Robo@({int(wx_r)},{int(wy_r)})",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

        # (4) Draw stored waypoint coords on map
        if self.cached_H is not None:
            Hinv = np.linalg.inv(self.cached_H)
            for wid, (wx, wy) in self.wp_coords.items():
                tmp = Hinv @ np.array([wx, wy, 1.0], dtype=np.float32)
                tmp /= tmp[2]
                px_img, py_img = tmp[0], tmp[1]
                cv2.circle(img, (int(px_img), int(py_img)), 5, (255,0,255), -1)
                cv2.putText(img, f"W{wid}",
                            (int(px_img)+5, int(py_img)+5),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),1)

        # (5) Driving control toward stored target
        twist = Twist()
        if self.cached_H is not None and wx_r is not None and wx_f is not None:
            wp_id = self.waypoints[self.wp_idx]
            # 최초 감지 시 좌표 저장
            if ids is not None and wp_id in ids.flatten() and wp_id not in self.wp_coords:
                i_w = list(ids.flatten()).index(wp_id)
                cw = corners[i_w][0]
                px_w, py_w = float(cw[:,0].mean()), float(cw[:,1].mean())
                self.wp_coords[wp_id] = pixel_to_world(self.cached_H, px_w, py_w)

            if wp_id in self.wp_coords:
                wx_w, wy_w = self.wp_coords[wp_id]
                # target 표시
                tmp = Hinv @ np.array([wx_w, wy_w, 1.0], dtype=np.float32)
                tmp /= tmp[2]
                px_t, py_t = tmp[0], tmp[1]
                cv2.circle(img, (int(px_t), int(py_t)), 5, (255,0,0), -1)

                # 제어 벡터 계산
                front_vec  = np.array([wx_f - wx_r, wy_f - wy_r])
                target_vec = np.array([wx_w - wx_r, wy_w - wy_r])
                dist_err   = math.hypot(target_vec[0], target_vec[1])
                cross = front_vec[0]*target_vec[1] - front_vec[1]*target_vec[0]
                dot   = float(np.dot(front_vec, target_vec))
                err   = math.degrees(math.atan2(cross, dot))

                # P-control
                K_ang, K_lin = 0.005, 0.002
                twist.angular.z = np.clip(K_ang * err, -0.3, 0.3)
                twist.linear.x  = np.clip(K_lin*dist_err*max(0, math.cos(math.radians(err))), 0, 0.2)

                # 도착 처리 & 순환
                if dist_err < 5.0:
                    twist.linear.x  = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info(f"WP {wp_id} 도착")
                    self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)

                cv2.putText(img,
                            f"Err:{err:.1f}° Dist:{dist_err:.1f}cm",
                            (10,120), cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)

        # (6) Publish & Display
        self.pub.publish(twist)
        cv2.imshow("win", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    if not os.path.exists(CALIB_PATH):
        print("camera_calibration.pkl 파일을 찾을 수 없습니다.")
        return
    with open(CALIB_PATH, 'rb') as f:
        calib = pickle.load(f)

    node = ArucoNavigator(calib)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
