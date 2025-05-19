#!/usr/bin/env python3
import math, pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node       import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg      import Int32

# ===== 설정 =====
CALIB_PATH    = '/home/addinedu/camera_calibration.pkl'
MAP_WIDTH_CM  = 200.0
MAP_HEIGHT_CM = 100.0
MARKER_SIZE_CM= 10.0
half = MARKER_SIZE_CM / 2.0

marker_world = {
    0: (half,                 half),
    1: (MAP_WIDTH_CM-half,    half),
    3: (half,                 MAP_HEIGHT_CM-half),
    4: (MAP_WIDTH_CM-half,    MAP_HEIGHT_CM-half),
}

WAYPOINT_IDS = [6,7,8]
PPM   = 5
MAP_W = int(MAP_WIDTH_CM * PPM)
MAP_H = int(MAP_HEIGHT_CM * PPM)

class ArucoNav(Node):
    def __init__(self):
        super().__init__('aruco_nav')
        # 퍼블리셔
        self.pose_pub   = self.create_publisher(PointStamped, '/robot_pose',  10)
        self.front_pub  = self.create_publisher(PointStamped, '/robot_front', 10)
        self.target_pub = self.create_publisher(Int32,        '/target_marker', 10)
        self.wp_pubs    = {
            wid: self.create_publisher(PointStamped, f'/waypoint_{wid}', 10)
            for wid in WAYPOINT_IDS
        }

        # 상태
        self.cached_H   = None
        self.wp_sent    = set()
        self.wp_coords  = {}
        self.last_target= None

        # 캘리브레이션
        with open(CALIB_PATH, 'rb') as f:
            calib = pickle.load(f)
        self.mtx, self.dist = calib['camera_matrix'], calib['dist_coeffs']

        # ArUco
        self.aruco_dict    = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params        = aruco.DetectorParameters()
        self.marker_length = MARKER_SIZE_CM

        # 윈도우
        cv2.namedWindow("win",    cv2.WINDOW_NORMAL);  cv2.resizeWindow("win",    640,480)
        cv2.namedWindow("tf_map", cv2.WINDOW_NORMAL);  cv2.resizeWindow("tf_map", int(MAP_W*0.75), int(MAP_H*0.75))

        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.create_timer(1/30.0, self.cb)

    def build_homography(self, corners, ids):
        if ids is None: return None
        img_pts, world_pts = [], []
        for i, mid in enumerate(ids.flatten()):
            if mid in marker_world:
                c = corners[i][0]
                img_pts.append([float(c[:,0].mean()), float(c[:,1].mean())])
                world_pts.append(marker_world[mid])
        if len(img_pts) < 4: return None
        H, _ = cv2.findHomography(np.array(img_pts,np.float32),
                                  np.array(world_pts,np.float32))
        return H

    def pixel_to_world(self, H, px, py):
        pt = np.array([px, py, 1.0], dtype=np.float32)
        w  = H @ pt; w /= w[2]
        return float(w[0]), float(w[1])  # cm

    def cb(self):
        ret, frame = self.cap.read()
        if not ret:
            cv2.waitKey(1); return

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

        # prepare error map
        err = {
            0: np.array([-5.5, -3.0], dtype=np.float32),
            1: np.array([+5.5, -3.0], dtype=np.float32),
            3: np.array([-5.5, +7.3], dtype=np.float32),
            4: np.array([+5.5, +7.3], dtype=np.float32),
        }

        # 5번 마커 정보 계산
        raw5 = corr5 = angle5 = None
        if ids is not None and self.cached_H is not None and 5 in ids.flatten():
            i5 = list(ids.flatten()).index(5)
            c5 = corners[i5][0]
            px, py = float(c5[:,0].mean()), float(c5[:,1].mean())

            # raw homography(cm)
            ox_r, oy_r = self.pixel_to_world(self.cached_H, px, py)
            raw5 = (ox_r, oy_r)

            # error bilinear
            s = ox_r / MAP_WIDTH_CM
            t = oy_r / MAP_HEIGHT_CM
            e00,e10 = err[0], err[1]
            e01,e11 = err[3], err[4]
            e_bot = (1-s)*e00 + s*e10
            e_top = (1-s)*e01 + s*e11
            e = (1-t)*e_bot + t*e_top

            # corrected(cm)
            x_corr, y_corr = ox_r - e[0], oy_r - e[1]
            corr5 = (x_corr, y_corr)

            # orientation error
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.mtx, self.dist)
            R, _ = cv2.Rodrigues(rvecs[i5][0])
            front3d = np.array([[0.0, self.marker_length/2, 0.0]]).T
            camf = (R @ front3d + tvecs[i5][0].reshape(3,1)).ravel()
            imgpts, _ = cv2.projectPoints(camf.reshape(1,3), np.zeros(3), np.zeros(3),
                                          self.mtx, self.dist)
            fx, fy = imgpts[0].ravel().astype(float)
            ox_f, oy_f = self.pixel_to_world(self.cached_H, fx, fy)
            angle5 = math.degrees(math.atan2(oy_f-oy_r, ox_f-ox_r))

            # publish pose & front
            p = PointStamped(); p.header.frame_id='map'; p.header.stamp=self.get_clock().now().to_msg()
            p.point.x, p.point.y, p.point.z = x_corr/100.0, y_corr/100.0, 0.0
            self.pose_pub.publish(p)
            f = PointStamped(); f.header=p.header
            x_corr_f = ox_f - e[0]; y_corr_f = oy_f - e[1]
            f.point.x, f.point.y, f.point.z = x_corr_f/100.0, y_corr_f/100.0, 0.0
            self.front_pub.publish(f)

        # waypoints 6,7,8
        if ids is not None and self.cached_H is not None:
            for wid in WAYPOINT_IDS:
                if wid in ids.flatten() and wid not in self.wp_sent:
                    idx = list(ids.flatten()).index(wid)
                    cw = corners[idx][0]
                    pxw, pyw = float(cw[:,0].mean()), float(cw[:,1].mean())
                    wx, wy = self.pixel_to_world(self.cached_H, pxw, pyw)
                    self.wp_coords[wid] = (wx, wy)
                    self.wp_sent.add(wid)
                    msg = PointStamped(); msg.header.frame_id='map'; msg.header.stamp=self.get_clock().now().to_msg()
                    msg.point.x, msg.point.y, msg.point.z = wx, wy, 0.0
                    self.wp_pubs[wid].publish(msg)

        # draw overlays
        y0 = 20
        # waypoints
        for wid in WAYPOINT_IDS:
            if wid in self.wp_coords:
                wx, wy = self.wp_coords[wid]
                cv2.putText(img, f"W{wid}:({wx:.1f},{wy:.1f})cm", (10,y0),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0),1)
                y0 += 20
        # marker5
        if raw5 is not None and corr5 is not None:
            cv2.putText(img, f"5 raw:({raw5[0]:.1f},{raw5[1]:.1f})cm", (10,y0),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,200),1); y0+=20
            cv2.putText(img, f"5 corr:({corr5[0]:.1f},{corr5[1]:.1f})cm", (10,y0),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),1); y0+=20
            cv2.putText(img, f"5 ang err:{angle5:.1f}deg", (10,y0),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,0),1)

        # display
        cv2.imshow("win", img)
        if self.cached_H is not None:
            S = np.array([[PPM,0,0],[0,-PPM,MAP_H],[0,0,1]], dtype=np.float32)
            tf_map = cv2.warpPerspective(img, S@self.cached_H, (MAP_W,MAP_H))
            cv2.imshow("tf_map", tf_map)

def main():
    rclpy.init()
    node = ArucoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
