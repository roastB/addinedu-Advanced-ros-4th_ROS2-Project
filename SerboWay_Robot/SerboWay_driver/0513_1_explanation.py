#!/usr/bin/env python3
import os, pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg      import Int32

# — 카메라 보정 파일 경로 (잘못되면 보정/좌표 변환 전체가 깨짐)
CALIB_PATH   = '/home/addinedu/camera_calibration.pkl'  

# — 호모그래피 기준 마커 월드 좌표(cm 단위)
#    맵 크기나 기준점을 바꾸려면 이 딕셔너리 값을 수정
marker_world = {0:(0,0), 1:(100,0), 4:(100,100), 3:(0,100)}

# — 웨이포인트로 등록할 마커 ID 리스트
#    ID 추가·삭제 시 여기를 바꿔주세요
WAYPOINT_IDS = [6,7,8]

class ArucoNav(Node):
    def __init__(self):
        super().__init__('aruco_nav')
        # 퍼블리셔: 로봇 위치·전방·타겟·웨이포인트 토픽
        self.pose_pub   = self.create_publisher(PointStamped, '/robot_pose',    10)
        self.front_pub  = self.create_publisher(PointStamped, '/robot_front',   10)
        self.target_pub = self.create_publisher(Int32,        '/target_marker', 10)
        self.wp_pubs    = {
            wid: self.create_publisher(PointStamped, f'/waypoint_{wid}', 10)
            for wid in WAYPOINT_IDS
        }
        self.wp_sent = set()

        # 카메라 보정값 로드
        with open(CALIB_PATH, 'rb') as f:
            calib = pickle.load(f)
        self.mtx, self.dist = calib['camera_matrix'], calib['dist_coeffs']

        # ArUco 검출 설정: 마커 패밀리·파라미터
        self.aruco_dict    = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params        = aruco.DetectorParameters()
        self.cached_H      = None
        # — 실제 마커 한 변 길이(cm)
        #    정확한 길이로 설정해야 자세 추정이 올바름
        self.marker_length = 10.0  

        # OpenCV 윈도우 및 카메라
        cv2.namedWindow("win", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("win", 640, 480)  # 영상 표시 해상도
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)  # 디바이스 인덱스
        if not self.cap.isOpened():
            self.get_logger().error("카메라 열기 실패")
            rclpy.shutdown()

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

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Undistort→Crop→Resize (보정 정확도에 영향)
        h, w = frame.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(
            self.mtx, self.dist, (w, h), 1, (w, h))
        und = cv2.undistort(frame, self.mtx, self.dist, None, new_mtx)
        x,y,w0,h0 = roi
        if all(v>0 for v in (x,y,w0,h0)):
            und = und[y:y+h0, x:x+w0]
        img = cv2.resize(und, (640, 480))  # 표시 크기

        # Detect & draw markers (민감도는 DetectorParameters 조절)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.params)
        aruco.drawDetectedMarkers(img, corners, ids)

        # Update homography
        H_new = self.build_homography(corners, ids)
        if H_new is not None:
            self.cached_H = H_new

        # Publish robot pose & front if marker 5 seen
        if ids is not None and self.cached_H is not None and 5 in ids.flatten():
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.mtx, self.dist)
            i5 = list(ids.flatten()).index(5)
            R, _ = cv2.Rodrigues(rvecs[i5][0])
            c5 = corners[i5][0]
            px_c = float(c5[:,0].mean()); py_c = float(c5[:,1].mean())
            wx_r, wy_r = self.pixel_to_world(self.cached_H, px_c, py_c)

            # front 3D→image→world
            front3d = np.array([[0.0, self.marker_length, 0.0]], dtype=np.float32)
            camf = (R @ front3d.T + tvecs[i5][0].reshape(3,1)).T
            imgpts, _ = cv2.projectPoints(
                camf, np.zeros(3), np.zeros(3),
                self.mtx, self.dist)
            fx, fy = imgpts[0].ravel().astype(int)
            wx_f, wy_f = self.pixel_to_world(self.cached_H, fx, fy)

            # publish pose
            p = PointStamped()
            p.header.frame_id = 'map'
            p.header.stamp    = self.get_clock().now().to_msg()
            p.point.x, p.point.y, p.point.z = wx_r, wy_r, 0.0
            self.pose_pub.publish(p)

            # publish front
            f = PointStamped(); f.header = p.header
            f.point.x, f.point.y, f.point.z = wx_f, wy_f, 0.0
            self.front_pub.publish(f)

            # draw arrow
            cv2.arrowedLine(img, (int(px_c),int(py_c)), (fx,fy),
                            (0,255,0), 2, tipLength=0.2)

        # Publish each waypoint once
        if ids is not None and self.cached_H is not None:
            for wid in WAYPOINT_IDS:
                if wid in ids.flatten() and wid not in self.wp_sent:
                    idx = list(ids.flatten()).index(wid)
                    cw = corners[idx][0]
                    px,py = float(cw[:,0].mean()), float(cw[:,1].mean())
                    wx, wy = self.pixel_to_world(self.cached_H, px, py)
                    msg = PointStamped()
                    msg.header.frame_id = 'map'
                    msg.header.stamp    = self.get_clock().now().to_msg()
                    msg.point.x, msg.point.y, msg.point.z = wx, wy, 0.0
                    self.wp_pubs[wid].publish(msg)
                    self.wp_sent.add(wid)

        cv2.imshow("win", img)

    def handle_keys(self):
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('6'), ord('7'), ord('8')):
            mid = int(chr(key))
            # 키 매핑: 눌린 키에 해당하는 ID 발행
            self.target_pub.publish(Int32(data=mid))
        elif key == ord('q'):
            rclpy.shutdown()

def main():
    rclpy.init()
    node = ArucoNav()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            node.process_frame()
            node.handle_keys()
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
