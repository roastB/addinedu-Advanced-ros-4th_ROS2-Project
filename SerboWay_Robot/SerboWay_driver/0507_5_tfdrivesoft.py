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

# ===== 설정 =====
CALIB_PATH   = '/home/addinedu/camera_calibration.pkl'
marker_world = {0:(0,0),1:(200,0),4:(200,100),3:(0,100)}  # cm 단위
WAYPOINTS    = [6,7,8]     # 3개 웨이포인트

# Bird’s-Eye View 스케일 & 맵 크기
PPM      = 5
MAP_W    = int(200 * PPM)
MAP_H    = int(100 * PPM)

# 제어 상수
K_ANG, K_LIN     = 0.005, 0.002
MAX_ANG, MAX_LIN = 0.15,  0.2    # 회전 속도 최대치 절반으로 감소
MIN_SPEED_F      = 0.1
ANGLE_UNIT       = 'deg'

def load_calibration(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} not found")
    with open(path,'rb') as f:
        c = pickle.load(f)
    return c['camera_matrix'], c['dist_coeffs']

def build_homography(corners, ids):
    if ids is None: return None
    img_pts, world_pts = [], []
    for i, mid in enumerate(ids.flatten()):
        if mid in marker_world:
            pts = corners[i][0]
            img_pts.append([float(pts[:,0].mean()), float(pts[:,1].mean())])
            world_pts.append(marker_world[mid])
    if len(img_pts) < 4: return None
    H,_ = cv2.findHomography(np.array(img_pts,np.float32),
                              np.array(world_pts,np.float32))
    return H

def pixel_to_world(H, px, py):
    pt = np.array([px,py,1.0], dtype=np.float32)
    w  = H @ pt; w /= w[2]
    return float(w[0]), float(w[1])

class ArucoNav(Node):
    def __init__(self, calib):
        super().__init__('aruco_nav')
        self.pub       = self.create_publisher(Twist,'/cmd_vel',10)
        self.mtx, self.dist = calib
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params     = aruco.DetectorParameters()
        self.cap        = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패')
            rclpy.shutdown()

        self.cached_H  = None
        self.wp_coords = {}
        self.route     = []
        self.route_idx = 0

        self.create_timer(1/30.0, self.cb)

    def update_route(self):
        self.route.clear()
        N = len(WAYPOINTS)
        for i in range(N):
            a = WAYPOINTS[i]
            b = WAYPOINTS[(i+1)%N]
            ca = self.wp_coords.get(a)
            cb = self.wp_coords.get(b)
            if ca and cb:
                # 시작 웨이포인트
                self.route.append((ca, f"W{a}"))
                # 중간점 3개: 25%, 50%, 75%
                for frac in (0.25, 0.50, 0.75):
                    mx = ca[0] + (cb[0]-ca[0])*frac
                    my = ca[1] + (cb[1]-ca[1])*frac
                    self.route.append(((mx,my), f"M{a}{b}_{int(frac*100)}"))

    def cb(self):
        ret, frame = self.cap.read()
        if not ret: return

        # 1) Undistort→Crop→Resize
        h,w = frame.shape[:2]
        new_mtx,roi = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),1,(w,h))
        und = cv2.undistort(frame,self.mtx,self.dist,None,new_mtx)
        x,y,w0,h0 = roi
        if all(v>0 for v in (x,y,w0,h0)):
            und = und[y:y+h0, x:x+w0]
        img = cv2.resize(und,(640,480))

        # 2) Detect ArUco & update H
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        corners,ids,_ = aruco.detectMarkers(gray,self.aruco_dict,parameters=self.params)
        aruco.drawDetectedMarkers(img,corners,ids)
        H_new = build_homography(corners,ids)
        if H_new is not None:
            self.cached_H = H_new

        # 3) Robot pose & front world coord
        wx_r=wy_r=wx_f=wy_f=None
        err=dist=None
        target=None
        if ids is not None and self.cached_H is not None and 5 in ids.flatten():
            rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners,10.0,self.mtx,self.dist)
            i5 = list(ids.flatten()).index(5)
            R,_ = cv2.Rodrigues(rvecs[i5][0])
            pts5 = corners[i5][0]
            px = float(pts5[:,0].mean()); py=float(pts5[:,1].mean())
            wx_r,wy_r = pixel_to_world(self.cached_H,px,py)

            front3 = np.array([[0,10,0]],dtype=np.float32)
            camf = (R@front3.T + tvecs[i5][0].reshape(3,1)).T
            imgpts,_=cv2.projectPoints(camf,np.zeros(3),np.zeros(3),self.mtx,self.dist)
            fx,fy = imgpts[0].ravel().astype(int)
            wx_f,wy_f = pixel_to_world(self.cached_H,fx,fy)

        # 4) Capture WP coords once
        if ids is not None and self.cached_H is not None:
            for wp in WAYPOINTS:
                if wp in ids.flatten() and wp not in self.wp_coords:
                    idx=list(ids.flatten()).index(wp)
                    pts = corners[idx][0]
                    px,py = float(pts[:,0].mean()),float(pts[:,1].mean())
                    self.wp_coords[wp]=pixel_to_world(self.cached_H,px,py)

        # 5) Update route
        if len(self.wp_coords)==len(WAYPOINTS) and self.cached_H is not None:
            self.update_route()

        # 6) Driving control
        twist = Twist()
        if wx_r is not None and wx_f is not None and self.route:
            (tx,ty),_ = self.route[self.route_idx]
            target=(tx,ty)
            fv=np.array([wx_f-wx_r,wy_f-wy_r])
            tv=np.array([tx-wx_r,ty-wy_r])
            dist=math.hypot(tv[0],tv[1])
            err=math.degrees(math.atan2(fv[0]*tv[1]-fv[1]*tv[0],np.dot(fv,tv)))

            twist.angular.z = np.clip(K_ANG*err,-MAX_ANG,MAX_ANG)
            sf = max(MIN_SPEED_F,math.cos(math.radians(err)))
            twist.linear.x  = np.clip(K_LIN*dist*sf,0,MAX_LIN)

            if dist<2.0:
                self.route_idx=(self.route_idx+1)%len(self.route)

        self.pub.publish(twist)

        # 7) map view
        cv2.imshow("map",img)

        # 8) topo view
        if self.cached_H is not None:
            S=np.array([[PPM,0,0],[0,-PPM,MAP_H],[0,0,1]],dtype=np.float32)
            M_tf=S@self.cached_H
            tf=cv2.warpPerspective(img,M_tf,(MAP_W,MAP_H))

            # draw all route pts
            for idx,(pt,lbl) in enumerate(self.route):
                x2=int(pt[0]*PPM); y2=int(MAP_H-pt[1]*PPM)
                if idx==(self.route_idx%len(self.route)):
                    col=(0,255,0)
                elif lbl.startswith("W"):
                    col=(255,0,255)
                else:
                    col=(0,255,255)
                cv2.circle(tf,(x2,y2),5,col,-1)
                cv2.putText(tf,lbl,(x2+5,y2+5),cv2.FONT_HERSHEY_SIMPLEX,0.6,col,2)
                cv2.putText(tf,f"({pt[0]:.0f},{pt[1]:.0f})",(x2+5,y2+25),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,col,1)
                if lbl.startswith("M"):
                    l=8
                    cv2.line(tf,(x2-l,y2-l),(x2+l,y2+l),col,2)
                    cv2.line(tf,(x2-l,y2+l),(x2+l,y2-l),col,2)

            # robot & errors
            if wx_r is not None:
                xr,yr=int(wx_r*PPM),int(MAP_H-wy_r*PPM)
                xf,yf=int(wx_f*PPM),int(MAP_H-wy_f*PPM)
                cv2.circle(tf,(xr,yr),5,(0,255,0),-1)
                cv2.arrowedLine(tf,(xr,yr),(xf,yf),(0,255,0),2,tipLength=0.2)
                if err is not None:
                    cv2.putText(tf,f"Err:{err:.1f}{ANGLE_UNIT}",(xr+10,yr-10),
                                cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
                if dist is not None:
                    cv2.putText(tf,f"D:{dist:.1f}cm",(xr+10,yr+10),
                                cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
            # highlight target
            if target is not None:
                xt,yt=int(target[0]*PPM),int(MAP_H-target[1]*PPM)
                cv2.circle(tf,(xt,yt),8,(0,0,255),2)

            cv2.imshow("tf_map",tf)

        if cv2.waitKey(1)&0xFF==ord('q'):
            rclpy.shutdown()

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    calib=load_calibration(CALIB_PATH)
    node=ArucoNav(calib)
    try: rclpy.spin(node)
    finally:
        node.destroy(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
