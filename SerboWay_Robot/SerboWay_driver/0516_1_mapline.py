#!/usr/bin/env python3
import math
import pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node         import Node
from geometry_msgs.msg   import PointStamped, Twist
from std_msgs.msg        import Int32
from sensor_msgs.msg     import LaserScan

# ===== 설정 =====
CALIB_PATH    = '/home/addinedu/camera_calibration.pkl'
MAP_WIDTH_CM  = 200.0
MAP_HEIGHT_CM = 100.0
MARKER_SIZE_CM = 10.0
half = MARKER_SIZE_CM / 2.0

marker_world = {
    0: (0   + half,  0   + half),
    1: (MAP_WIDTH_CM - half,  0   + half),
    3: (0   + half,  MAP_HEIGHT_CM - half),
    4: (MAP_WIDTH_CM - half,  MAP_HEIGHT_CM - half),
}

WAYPOINT_IDS = [6,7,8]
PPM   = 5
MAP_W = int(MAP_WIDTH_CM  * PPM)
MAP_H = int(MAP_HEIGHT_CM * PPM)

class ArucoNav(Node):
    def __init__(self):
        super().__init__('aruco_nav')

        # 퍼블리셔
        self.pose_pub   = self.create_publisher(PointStamped, '/robot_pose',    10)
        self.front_pub  = self.create_publisher(PointStamped, '/robot_front',   10)
        self.target_pub = self.create_publisher(Int32,        '/target_marker', 10)
        self.wp_pubs    = {
            wid: self.create_publisher(PointStamped, f'/waypoint_{wid}', 10)
            for wid in WAYPOINT_IDS
        }
        self.wp_sent     = set()
        self.wp_coords   = {}
        self.last_target = None
        self.start_pose  = None   # 5번 좌표 고정용
        self.cached_H    = None
        self.current_pose = None  # 매 프레임 갱신되는 로봇 현재 좌표

        # 카메라 캘리브레이션
        with open(CALIB_PATH, 'rb') as f:
            calib = pickle.load(f)
        self.mtx, self.dist = calib['camera_matrix'], calib['dist_coeffs']

        # ArUco 설정
        self.aruco_dict    = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params        = aruco.DetectorParameters()
        self.marker_length = MARKER_SIZE_CM

        # OpenCV 창
        cv2.namedWindow("win",    cv2.WINDOW_NORMAL);  cv2.resizeWindow("win",  640, 480)
        cv2.namedWindow("tf_map", cv2.WINDOW_NORMAL);  cv2.resizeWindow("tf_map", int(MAP_W*0.75), int(MAP_H*0.75))

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

        # undistort → crop → resize
        h, w = frame.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        und = cv2.undistort(frame, self.mtx, self.dist, None, new_mtx)
        x,y,w0,h0 = roi
        if all(v>0 for v in (x,y,w0,h0)):
            und = und[y:y+h0, x:x+w0]
        img = cv2.resize(und, (640,480))

        # detect & draw
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        aruco.drawDetectedMarkers(img, corners, ids)

        # homography 갱신
        H_new = self.build_homography(corners, ids)
        if H_new is not None:
            self.cached_H = H_new

        wx_r = wy_r = wx_f = wy_f = None

        # 5번 마커(로봇) pose & front publish
        if ids is not None and self.cached_H is not None and 5 in ids.flatten():
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.mtx, self.dist)
            i5 = list(ids.flatten()).index(5)
            R,_ = cv2.Rodrigues(rvecs[i5][0])
            c5 = corners[i5][0]
            px, py = float(c5[:,0].mean()), float(c5[:,1].mean())
            wx_r, wy_r = self.pixel_to_world(self.cached_H, px, py)
            self.current_pose = (wx_r, wy_r)  # 현재 좌표 갱신

            # front 점 계산
            front3d = np.array([[0.0, self.marker_length, 0.0]], dtype=np.float32)
            camf = (R @ front3d.T + tvecs[i5][0].reshape(3,1)).T
            imgpts, _ = cv2.projectPoints(camf, np.zeros(3), np.zeros(3),
                                          self.mtx, self.dist)
            fx, fy = imgpts[0].ravel().astype(int)
            wx_f, wy_f = self.pixel_to_world(self.cached_H, fx, fy)

            # publish
            p = PointStamped(); p.header.frame_id='map'; p.header.stamp=self.get_clock().now().to_msg()
            p.point.x, p.point.y, p.point.z = wx_r, wy_r, 0.0
            self.pose_pub.publish(p)
            f = PointStamped(); f.header=p.header
            f.point.x, f.point.y, f.point.z = wx_f, wy_f, 0.0
            self.front_pub.publish(f)
            cv2.arrowedLine(img, (int(px),int(py)), (fx,fy),(0,255,0),2,tipLength=0.2)

        # waypoint publish (한 번만)
        if ids is not None and self.cached_H is not None:
            for wid in WAYPOINT_IDS:
                if wid in ids.flatten() and wid not in self.wp_sent:
                    idx = list(ids.flatten()).index(wid)
                    cw = corners[idx][0]
                    px, py = float(cw[:,0].mean()), float(cw[:,1].mean())
                    wx, wy = self.pixel_to_world(self.cached_H, px, py)
                    msg = PointStamped(); msg.header.frame_id='map'; msg.header.stamp=self.get_clock().now().to_msg()
                    msg.point.x, msg.point.y, msg.point.z = wx, wy, 0.0
                    self.wp_pubs[wid].publish(msg)
                    self.wp_coords[wid] = (wx, wy)
                    self.wp_sent.add(wid)

        # 키 처리: 6/7/8 → 목표 설정 & start_pose 고정
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('6'), ord('7'), ord('8')):
            if self.current_pose is not None:
                self.start_pose = self.current_pose
                self.last_target = int(chr(key))
                self.target_pub.publish(Int32(data=self.last_target))
        elif key == ord('q'):
            rclpy.shutdown()

        cv2.imshow("win", img)

        # bird’s-eye view
        if self.cached_H is not None:
            S      = np.array([[PPM,0,0],[0,-PPM,MAP_H],[0,0,1]], np.float32)
            tf_map = cv2.warpPerspective(img, S @ self.cached_H, (MAP_W,MAP_H))

            # waypoints
            for wid,(wx,wy) in self.wp_coords.items():
                x2, y2 = int(wx*PPM), int(MAP_H - wy*PPM)
                cv2.circle(tf_map,(x2,y2),5,(0,255,0),-1)
                cv2.putText(tf_map, f"W{wid}", (x2+5,y2+5),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)

            # robot pose
            if wx_r is not None:
                xr, yr = int(wx_r*PPM), int(MAP_H - wy_r*PPM)
                xf, yf = int(wx_f*PPM), int(MAP_H - wy_f*PPM)
                cv2.circle(tf_map,(xr,yr),5,(0,255,0),-1)
                cv2.arrowedLine(tf_map,(xr,yr),(xf,yf),(0,255,0),2,tipLength=0.2)

            # start_pose → target 선 그리기 (고정된 start_pose 사용)
            if self.start_pose is not None and self.last_target in self.wp_coords:
                sx, sy = self.start_pose
                tx, ty = self.wp_coords[self.last_target]
                x0, y0 = int(sx*PPM), int(MAP_H - sy*PPM)
                x1, y1 = int(tx*PPM), int(MAP_H - ty*PPM)
                cv2.line(tf_map, (x0,y0), (x1,y1), (255,0,0), 2)

                # 도착 판정
                dist = math.hypot(self.current_pose[0] - tx, self.current_pose[1] - ty)
                if dist < MARKER_SIZE_CM:
                    self.start_pose = None
                    self.last_target = None

            cv2.imshow("tf_map", tf_map)

def main():
    rclpy.init()
    node = ArucoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
