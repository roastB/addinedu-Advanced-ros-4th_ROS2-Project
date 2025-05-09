#!/usr/bin/env python3
import os
import pickle
import cv2
import cv2.aruco as aruco
import numpy as np

CALIB_PATH = '/home/addinedu/camera_calibration.pkl'
marker_world = {0:(0,0),1:(200,0),4:(200,100),3:(0,100)}
waypoint_ids = [6,7,8]
PIXELS_PER_CM = 5
MAP_W = int(200 * PIXELS_PER_CM)  # 1000
MAP_H = int(100 * PIXELS_PER_CM)  # 500

def load_calibration(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} not found")
    with open(path,'rb') as f:
        calib = pickle.load(f)
    return calib['camera_matrix'], calib['dist_coeffs']

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
        np.array(img_pts,dtype=np.float32),
        np.array(world_pts,dtype=np.float32))
    return H

def main():
    mtx, dist = load_calibration(CALIB_PATH)
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("카메라 열기 실패"); return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    params     = aruco.DetectorParameters()
    cached_H   = None

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 1) Undistort → Crop → Resize
        h, w = frame.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h),1,(w,h))
        un = cv2.undistort(frame, mtx, dist, None, new_mtx)
        x, y, w0, h0 = roi
        if all(v>0 for v in (x,y,w0,h0)):
            un = un[y:y+h0, x:x+w0]
        img = cv2.resize(un, (640,480))

        # 2) Detect & Homography
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
        aruco.drawDetectedMarkers(img, corners, ids)
        H_new = build_homography(corners, ids)
        if H_new is not None:
            cached_H = H_new

        # 3) Require all four map markers
        if (ids is None or cached_H is None or
            any(mid not in ids.flatten() for mid in (0,1,4,3))):
            blank = np.zeros((MAP_H, MAP_W, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for map markers...",
                        (50, MAP_H//2), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0,0,255), 2)
            cv2.imshow("Top-Down Map", blank)
            if cv2.waitKey(1)&0xFF==ord('q'): break
            continue

        # 4) src/dst for PerspectiveTransform
        src_pts = []
        for mid in (0,1,4,3):
            idx = list(ids.flatten()).index(mid)
            c = corners[idx][0]
            src_pts.append([c[:,0].mean(), c[:,1].mean()])
        src = np.array(src_pts, dtype=np.float32)

        # 🔄 여기서 Y축을 뒤집어 줍니다.
        dst = np.array([
            [marker_world[0][0]*PIXELS_PER_CM, MAP_H - marker_world[0][1]*PIXELS_PER_CM],
            [marker_world[1][0]*PIXELS_PER_CM, MAP_H - marker_world[1][1]*PIXELS_PER_CM],
            [marker_world[4][0]*PIXELS_PER_CM, MAP_H - marker_world[4][1]*PIXELS_PER_CM],
            [marker_world[3][0]*PIXELS_PER_CM, MAP_H - marker_world[3][1]*PIXELS_PER_CM],
        ], dtype=np.float32)

        M = cv2.getPerspectiveTransform(src, dst)
        topdown = cv2.warpPerspective(img, M, (MAP_W, MAP_H))

        # 5) Project waypoints 6,7,8
        wp_map_pts = {}
        for wp in waypoint_ids:
            if wp in ids.flatten():
                i = list(ids.flatten()).index(wp)
                c = corners[i][0]
                pxm = float(c[:,0].mean()); pym = float(c[:,1].mean())
                pt = np.array([[[pxm,pym]]], dtype=np.float32)
                warped = cv2.perspectiveTransform(pt, M)[0][0]
                wp_map_pts[wp] = (int(warped[0]), int(warped[1]))

        # 6) Build route + midpoints
        route = []
        for i,a in enumerate(waypoint_ids):
            b = waypoint_ids[(i+1)%len(waypoint_ids)]
            if a in wp_map_pts and b in wp_map_pts:
                route.append((wp_map_pts[a], f"W{a}"))
                xa,ya = wp_map_pts[a]; xb,yb = wp_map_pts[b]
                route.append((( (xa+xb)//2, (ya+yb)//2 ), f"M{a}{b}"))

        # 7) Draw on topdown (no original)
        for pt, label in route:
            x2, y2 = pt
            color = (255,0,255) if label.startswith("W") else (0,255,255)
            cv2.circle(topdown, (x2, y2), 5, color, -1)
            cv2.putText(topdown, label, (x2+5, y2+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow("Top-Down Map", topdown)
        if cv2.waitKey(1)&0xFF==ord('q'): break

    cap.release()
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
