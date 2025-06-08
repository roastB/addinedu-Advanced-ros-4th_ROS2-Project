#!/usr/bin/env python3
import math
import pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32

# ===== 설정 =====
CALIB_PATH    = '/home/addinedu/camera_calibration.pkl'
MAP_WIDTH_CM  = 200.0
MAP_HEIGHT_CM = 100.0
MARKER_SIZE_CM = 10.0
half = MARKER_SIZE_CM / 2.0

marker_world = {
    0: (0 + half, 0 + half),
    1: (MAP_WIDTH_CM - half, 0 + half),
    3: (0 + half, MAP_HEIGHT_CM - half),
    4: (MAP_WIDTH_CM - half, MAP_HEIGHT_CM - half),
}

WAYPOINT_IDS = [6, 7]
PPM   = 5
MAP_W = int(MAP_WIDTH_CM * PPM)
MAP_H = int(MAP_HEIGHT_CM * PPM)

class ArucoNav(Node):
    def __init__(self):
        super().__init__('aruco_nav')

        self.pose_pub   = self.create_publisher(PointStamped, '/robot_pose', 10)
        self.front_pub  = self.create_publisher(PointStamped, '/robot_front', 10)
        self.target_pub = self.create_publisher(Int32, '/target_marker', 10)
        self.wp_pubs = {
            wid: self.create_publisher(PointStamped, f'/waypoint_{wid}', 10)
            for wid in WAYPOINT_IDS
        }

        self.wp_coords = {}
        self.last_target = None
        self.start_pose  = None
        self.cached_H    = None
        self.current_pose = None

        with open(CALIB_PATH, 'rb') as f:
            calib = pickle.load(f)
        self.mtx, self.dist = calib['camera_matrix'], calib['dist_coeffs']

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params = aruco.DetectorParameters()
        self.marker_length = MARKER_SIZE_CM

        cv2.namedWindow("win", cv2.WINDOW_NORMAL);  cv2.resizeWindow("win", 640, 480)
        cv2.namedWindow("tf_map", cv2.WINDOW_NORMAL);  cv2.resizeWindow("tf_map", int(MAP_W*1.25), int(MAP_H*1.25))

        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
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
        H, _ = cv2.findHomography(np.array(img_pts,np.float32),
                                  np.array(world_pts,np.float32))
        return H

    def pixel_to_world(self, H, px, py):
        pt = np.array([px, py, 1.0], dtype=np.float32)
        w  = H @ pt; w /= w[2]
        return float(w[0]), float(w[1])

    def cb(self):
        ret, frame = self.cap.read()
        if not ret:
            cv2.waitKey(1); return

        h, w = frame.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        und = cv2.undistort(frame, self.mtx, self.dist, None, new_mtx)
        x,y,w0,h0 = roi
        if all(v>0 for v in (x,y,w0,h0)):
            und = und[y:y+h0, x:x+w0]
        img = cv2.resize(und, (640,480))

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        aruco.drawDetectedMarkers(img, corners, ids)

        H_new = self.build_homography(corners, ids)
        if H_new is not None:
            self.cached_H = H_new

        wx_r = wy_r = wx_f = wy_f = None

        if ids is not None and self.cached_H is not None and 5 in ids.flatten():
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.mtx, self.dist)
            i5 = list(ids.flatten()).index(5)
            R,_ = cv2.Rodrigues(rvecs[i5][0])
            c5 = corners[i5][0]
            px, py = float(c5[:,0].mean()), float(c5[:,1].mean())
            wx_r, wy_r = self.pixel_to_world(self.cached_H, px, py)
            self.current_pose = (wx_r, wy_r)

            front3d = np.array([[0.0, self.marker_length, 0.0]], dtype=np.float32)
            camf = (R @ front3d.T + tvecs[i5][0].reshape(3,1)).T
            imgpts, _ = cv2.projectPoints(camf, np.zeros(3), np.zeros(3), self.mtx, self.dist)
            fx, fy = imgpts[0].ravel().astype(int)
            wx_f, wy_f = self.pixel_to_world(self.cached_H, fx, fy)

            p = PointStamped(); p.header.frame_id='map'; p.header.stamp=self.get_clock().now().to_msg()
            p.point.x, p.point.y = wx_r, wy_r
            self.pose_pub.publish(p)

            f = PointStamped(); f.header = p.header
            f.point.x, f.point.y = wx_f, wy_f
            self.front_pub.publish(f)

            cv2.arrowedLine(img, (int(px), int(py)), (fx, fy), (0,255,0), 2, tipLength=0.2)

        # ⬇ 마커 6,7 지속 publish
        if ids is not None and self.cached_H is not None:
            for wid in WAYPOINT_IDS:
                if wid in ids.flatten():
                    idx = list(ids.flatten()).index(wid)
                    cw = corners[idx][0]
                    px, py = float(cw[:,0].mean()), float(cw[:,1].mean())
                    wx, wy = self.pixel_to_world(self.cached_H, px, py)

                    msg = PointStamped()
                    msg.header.frame_id = 'map'
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.point.x = wx
                    msg.point.y = wy
                    msg.point.z = 0.0

                    self.wp_coords[wid] = (wx, wy)
                    self.wp_pubs[wid].publish(msg)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('6'), ord('7')):
            if self.current_pose is not None:
                self.start_pose = self.current_pose
                self.last_target = int(chr(key))
                self.target_pub.publish(Int32(data=self.last_target))
        elif key == ord('q'):
            rclpy.shutdown()

        cv2.imshow("win", img)

        if self.cached_H is not None:
            S = np.array([[PPM,0,0],[0,-PPM,MAP_H],[0,0,1]], np.float32)
            tf_map = cv2.warpPerspective(img, S @ self.cached_H, (MAP_W, MAP_H))

            # === tf_map 시각화 ===
            key_points = {
                'T1':   (70, 67),
                'T2':   (94, 67),
                'T3':   (118, 67),
                'DROP': (32, 67),
                'PICK': (144, 29),
                'HOME': (92, 11),
            }
            for label, (x, y) in key_points.items():
                x_px, y_px = int(x * PPM), int(MAP_H - y * PPM)
                cv2.circle(tf_map, (x_px, y_px), 6, (0, 0, 255), -1)
                cv2.putText(tf_map, label, (x_px + 6, y_px - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            corner_points = {
                'P4': (42, 71),
                'P3': (144, 71),
                'P2': (138, 31),
                'P1': (44, 31),
            }
            for label, (x, y) in corner_points.items():
                x_px, y_px = int(x * PPM), int(MAP_H - y * PPM)
                cv2.circle(tf_map, (x_px, y_px), 6, (255, 255, 0), -1)
                cv2.putText(tf_map, label, (x_px + 6, y_px - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            inner_rect = [(52, 61), (134, 61), (130, 41), (54, 41)]
            inner_pts = [(int(x * PPM), int(MAP_H - y * PPM)) for (x, y) in inner_rect]
            for i in range(len(inner_pts)):
                cv2.line(tf_map, inner_pts[i], inner_pts[(i+1)%4], (200, 200, 200), 2)

            cv2.imshow("tf_map", tf_map)

def main():
    rclpy.init()
    node = ArucoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
