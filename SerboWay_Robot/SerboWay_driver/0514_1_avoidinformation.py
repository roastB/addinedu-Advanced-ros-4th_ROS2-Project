#!/usr/bin/env python3
import math, pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg      import Int32

CALIB_PATH   = '/home/addinedu/camera_calibration.pkl'
MAP_W_CM     = 100.0
MAP_H_CM     = 100.0
MARKER_SIZE  = 10.0
half         = MARKER_SIZE/2
WAYPOINT_IDS = [6,7,8]

marker_world = {
    0: (half,             half),
    1: (MAP_W_CM-half,    half),
    3: (half,             MAP_H_CM-half),
    4: (MAP_W_CM-half,    MAP_H_CM-half),
}

class ArUcoNav(Node):
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

        # 카메라 보정 로드
        with open(CALIB_PATH, 'rb') as f:
            calib = pickle.load(f)
        self.mtx, self.dist = calib['camera_matrix'], calib['dist_coeffs']

        # ArUco 설정
        self.aruco_dict    = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params        = aruco.DetectorParameters()
        self.marker_length = MARKER_SIZE

        # OpenCV 창
        cv2.namedWindow("win", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("win", 640, 480)
        cv2.namedWindow("tf_map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("tf_map", 480, 480)
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

        # ArUco 처리 타이머
        self.create_timer(1/30.0, self.cb)

    def cb(self):
        ret, frame = self.cap.read()
        if not ret:
            cv2.waitKey(1); return

        # 언디스토션 → 크롭 → 리사이즈
        h,w = frame.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h),1,(w,h))
        und = cv2.undistort(frame, self.mtx, self.dist, None, new_mtx)
        x,y,w0,h0 = roi
        if all(v>0 for v in (x,y,w0,h0)):
            und = und[y:y+h0, x:x+w0]
        img = cv2.resize(und, (640,480))

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        aruco.drawDetectedMarkers(img, corners, ids)

        # 호모그래피
        H = self.build_homography(corners, ids)
        if H is not None and ids is not None and 5 in ids.flatten():
            i5 = list(ids.flatten()).index(5)
            c5 = corners[i5][0]
            px,py = float(c5[:,0].mean()), float(c5[:,1].mean())
            wx,wy = self.pixel_to_world(H, px, py)

            # 전방 화살표
            rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.mtx, self.dist)
            R,_ = cv2.Rodrigues(rvecs[i5][0])
            front3d = np.array([[0.,self.marker_length,0.]],dtype=np.float32)
            camf = (R @ front3d.T + tvecs[i5][0].reshape(3,1)).T
            imgpts,_ = cv2.projectPoints(camf, np.zeros(3), np.zeros(3),
                                          self.mtx, self.dist)
            fx,fy = imgpts[0].ravel().astype(int)
            wfx,wfy = self.pixel_to_world(H, fx, fy)

            # 퍼블리시
            ts = self.get_clock().now().to_msg()
            p = PointStamped(); p.header.frame_id='map'; p.header.stamp=ts
            p.point.x,p.point.y,p.point.z = wx,wy,0
            self.pose_pub.publish(p)
            f = PointStamped(); f.header = p.header
            f.point.x,f.point.y,f.point.z = wfx,wfy,0
            self.front_pub.publish(f)

            # 화면 화살표
            cv2.arrowedLine(img, (int(px),int(py)), (fx,fy), (0,255,0), 2, tipLength=0.2)

        # 웨이포인트 퍼블리시
        if H is not None and ids is not None:
            for wid in WAYPOINT_IDS:
                if wid in ids.flatten():
                    c = corners[list(ids.flatten()).index(wid)][0]
                    px,py = float(c[:,0].mean()), float(c[:,1].mean())
                    wx,wy = self.pixel_to_world(H, px, py)
                    msg = PointStamped(); msg.header.frame_id='map'
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.point.x,msg.point.y,msg.point.z = wx,wy,0
                    self.wp_pubs[wid].publish(msg)

        cv2.imshow("win", img)
        if H is not None:
            bird = cv2.warpPerspective(img,
                       np.array([[PPM,0,0],[0,-PPM,PPM*MAP_H_CM],[0,0,1]],np.float32) @ H,
                       (int(PPM*MAP_W_CM), int(PPM*MAP_H_CM)))
            cv2.imshow("tf_map", bird)

    def build_homography(self, corners, ids):
        if ids is None: return None
        img_pts, world_pts = [], []
        for i,mid in enumerate(ids.flatten()):
            if mid in marker_world:
                c = corners[i][0]
                img_pts.append([c[:,0].mean(), c[:,1].mean()])
                world_pts.append(marker_world[mid])
        if len(img_pts)<4: return None
        H,_ = cv2.findHomography(np.array(img_pts,np.float32),
                                  np.array(world_pts,np.float32))
        return H

    def pixel_to_world(self, H, x,y):
        pt = np.array([x,y,1.],dtype=np.float32)
        w = H@pt; w/=w[2]
        return float(w[0]), float(w[1])

def main():
    rclpy.init()
    node = ArUcoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
