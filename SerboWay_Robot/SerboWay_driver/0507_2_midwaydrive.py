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

CALIB_PATH = '/home/addinedu/camera_calibration.pkl'
marker_world = {0:(0,0),1:(200,0),4:(200,100),3:(0,100)}

def build_homography(corners, ids):
    if ids is None: return None
    img_pts, world_pts = [], []
    for i, mid in enumerate(ids.flatten()):
        if mid in marker_world:
            c = corners[i][0]
            img_pts.append([float(c[:,0].mean()), float(c[:,1].mean())])
            world_pts.append(marker_world[mid])
    if len(img_pts) < 4: return None
    H, _ = cv2.findHomography(
        np.array(img_pts, dtype=np.float32),
        np.array(world_pts, dtype=np.float32)
    )
    return H

def pixel_to_world(H, px, py):
    pt = np.array([px, py, 1.0], dtype=np.float32)
    w  = H @ pt; w /= w[2]
    return float(w[0]), float(w[1])

class ArucoNav(Node):
    def __init__(self, calib):
        super().__init__('aruco_nav')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mtx, self.dist = calib['camera_matrix'], calib['dist_coeffs']
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params     = aruco.DetectorParameters()
        self.cached_H   = None

        self.waypoints = [6,7,8]
        self.wp_coords = {}
        self.route = []
        self.route_idx = 0

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패')
            rclpy.shutdown()

        self.create_timer(1/30.0, self.cb)

    def update_route(self):
        self.route.clear()
        for i, a in enumerate(self.waypoints):
            b = self.waypoints[(i+1) % len(self.waypoints)]
            ca = self.wp_coords.get(a)
            cb = self.wp_coords.get(b)
            if ca and cb:
                self.route.append((ca, f"W{a}"))
                mid = ((ca[0]+cb[0])/2.0, (ca[1]+cb[1])/2.0)
                self.route.append((mid, f"M{a}{b}"))

    def cb(self):
        ret, frame = self.cap.read()
        if not ret: return

        # 1) Undistort → Crop → Resize
        h, w = frame.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(
            self.mtx, self.dist, (w, h), 1, (w, h))
        und = cv2.undistort(frame, self.mtx, self.dist, None, new_mtx)
        x, y, w0, h0 = roi
        if all(v > 0 for v in [x, y, w0, h0]):
            und = und[y:y+h0, x:x+w0]
        img = cv2.resize(und, (640, 480))

        # 2) Detect markers & update H
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.params)
        aruco.drawDetectedMarkers(img, corners, ids)
        H_new = build_homography(corners, ids)
        if H_new is not None:
            self.cached_H = H_new

        # 3) Robot pose → front, wx_r, wy_r
        wx_r = wy_r = wx_f = wy_f = None
        if ids is not None and self.cached_H is not None and 5 in ids.flatten():
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, 10.0, self.mtx, self.dist)
            i5 = list(ids.flatten()).index(5)
            R, _ = cv2.Rodrigues(rvecs[i5][0])
            c5 = corners[i5][0]
            px_c = float(c5[:,0].mean()); py_c = float(c5[:,1].mean())
            wx_r, wy_r = pixel_to_world(self.cached_H, px_c, py_c)

            front3 = np.array([[0.0, 10.0, 0.0]], dtype=np.float32)
            cam_f = (R @ front3.T + tvecs[i5][0].reshape(3,1)).T
            pts, _ = cv2.projectPoints(
                cam_f, np.zeros(3), np.zeros(3), self.mtx, self.dist)
            fx, fy = pts[0].ravel().astype(int)
            cv2.arrowedLine(img, (int(px_c),int(py_c)), (fx,fy),
                            (0,255,0),2,tipLength=0.2)
            wx_f, wy_f = pixel_to_world(self.cached_H, fx, fy)

        # 4) Capture waypoint coords once
        if ids is not None and self.cached_H is not None:
            for wp in self.waypoints:
                if wp in ids.flatten() and wp not in self.wp_coords:
                    i_w = list(ids.flatten()).index(wp)
                    cw = corners[i_w][0]
                    px_w, py_w = float(cw[:,0].mean()), float(cw[:,1].mean())
                    self.wp_coords[wp] = pixel_to_world(self.cached_H, px_w, py_w)

        # 5) Update route if all waypoints seen
        if len(self.wp_coords) == len(self.waypoints) and self.cached_H is not None:
            self.update_route()

        # 6) Draw map + route
        if self.cached_H is not None:
            Hinv = np.linalg.inv(self.cached_H)
            for idx, ((wx, wy), label) in enumerate(self.route):
                tmp = Hinv @ np.array([wx, wy, 1.0], dtype=np.float32)
                tmp /= tmp[2]
                xi, yi = int(tmp[0]), int(tmp[1])
                if idx == self.route_idx:
                    col = (0,255,0)
                else:
                    col = (255,0,255) if label.startswith("W") else (0,255,255)
                cv2.circle(img, (xi, yi), 6, col, -1)
                cv2.putText(img, label, (xi+5, yi+5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

        # 7) Driving control along route
        twist = Twist()
        if (self.cached_H is not None and wx_r is not None
            and wx_f is not None and len(self.route) > 0):
            (tx, ty), _ = self.route[self.route_idx]
            fv = np.array([wx_f - wx_r, wy_f - wy_r])
            tv = np.array([tx    - wx_r, ty    - wy_r])
            dist = math.hypot(tv[0], tv[1])
            cross = fv[0]*tv[1] - fv[1]*tv[0]
            dot   = float(np.dot(fv, tv))
            err   = math.degrees(math.atan2(cross, dot))

            twist.angular.z = np.clip(0.005 * err, -0.3, 0.3)
            twist.linear.x  = np.clip(0.002 * dist * max(0, math.cos(math.radians(err))),
                                      0, 0.2)

            if dist < 5.0:
                self.route_idx = (self.route_idx + 1) % len(self.route)

        self.pub.publish(twist)
        cv2.imshow("map", img)
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

    node = ArucoNav(calib)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
