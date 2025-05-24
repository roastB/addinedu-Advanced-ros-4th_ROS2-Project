#!/usr/bin/env python3
import math
import pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg      import Empty
import heapq

# Calibration and map settings
CALIB_PATH     = '/home/addinedu/camera_calibration.pkl'
MAP_WIDTH_CM   = 200.0
MAP_HEIGHT_CM  = 100.0
MARKER_SIZE_CM = 10.0
half = MARKER_SIZE_CM / 2.0
PPM = 5

# World marker coordinates (cm) for homography
marker_world = {
    0: (half,               half),
    1: (MAP_WIDTH_CM-half,  half),
    3: (half,               MAP_HEIGHT_CM-half),
    4: (MAP_WIDTH_CM-half,  MAP_HEIGHT_CM-half),
}

# ID→coord for bird’s-eye plotting & graph
id_to_coord = {
    11:(44,31), 12:(138,31), 13:(144,67), 14:(42,71),
    21:(70,67), 22:(94,67),  23:(118,67),
    31:(144,29),32:(32,67),
    97:(92,69), 98:(92,31),  99:(92,15),
}

def build_graph():
    node_coords = {
        'P1': id_to_coord[11], 'P2': id_to_coord[12],
        'P3': id_to_coord[13], 'P4': id_to_coord[14],
        'T1': id_to_coord[21], 'T2': id_to_coord[22], 'T3': id_to_coord[23],
        'Pick': id_to_coord[31], 'Drop': id_to_coord[32],
        'mid': id_to_coord[98], 'Home': id_to_coord[99], 'upmid' : id_to_coord[97]
    }
    adj = {
        'P4':['P1','upmid'], 'P3':['P2','upmid'], 'P2':['P3','mid'], 'P1':['P3','mid'],
        'mid':['P1','P2'], 'upmid':['P3', 'P4'], 'Home':['mid'], 'Pick':['P2'], 'Drop':['P4'],
        'T1':['P3','P4'],'T2':['P3','P4'],'T3':['P3','P4'],
    }
    # make bidirectional
    for u,nbrs in list(adj.items()):
        for v in nbrs:
            adj.setdefault(v,[])
            if u not in adj[v]:
                adj[v].append(u)
    name_to_id = {
        'P1':11,'P2':12,'P3':13,'P4':14,
        'T1':21,'T2':22,'T3':23,
        'Pick':31,'Drop':32,
        'mid':98,'Home':99,
    }
    id_to_name = {v:k for k,v in name_to_id.items()}
    return node_coords, adj, name_to_id, id_to_name

def heuristic(a,b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def astar_graph(node_coords, adj, start, goal):
    open_set = []
    heapq.heappush(open_set, (heuristic(node_coords[start], node_coords[goal]), 0, start, None))
    came_from = {}
    gscore = {start: 0}
    while open_set:
        f, cost, current, parent = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while parent:
                path.append(parent)
                parent = came_from[parent]
            return path[::-1]
        if current in came_from:
            continue
        came_from[current] = parent
        for neigh in adj[current]:
            d = heuristic(node_coords[current], node_coords[neigh])
            tg = cost + d
            if tg < gscore.get(neigh, float('inf')):
                gscore[neigh] = tg
                heapq.heappush(open_set, (tg + heuristic(node_coords[neigh], node_coords[goal]), tg, neigh, current))
    return []

class ArucoNavCommander(Node):
    def __init__(self):
        super().__init__('aruco_nav_commander')

        # Publishers
        self.pose_pub    = self.create_publisher(PointStamped, '/robot_pose',  10)
        self.front_pub   = self.create_publisher(PointStamped, '/robot_front', 10)
        self.target_pub  = self.create_publisher(PointStamped, '/target_point',10)
        self.arrived_pub = self.create_publisher(Empty,        '/arrived',     10)

        # Load camera calibration
        with open(CALIB_PATH,'rb') as f:
            calib = pickle.load(f)
        self.mtx, self.dist    = calib['camera_matrix'], calib['dist_coeffs']
        self.aruco_dict        = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params            = aruco.DetectorParameters()
        self.marker_length     = MARKER_SIZE_CM
        self.cap               = cv2.VideoCapture(2, cv2.CAP_V4L2)

        # A* graph setup
        self.node_coords, self.adj, self.name_to_id, self.id_to_name = build_graph()
        self.current_pose  = None
        self.path_queue    = []
        self.active_target = None
        self.cached_H      = None

        # Static key→ID mapping
        self.key_to_dest_id = {1:21, 2:22, 3:23, 8:31, 9:32, 0:99}

        # Visualization windows
        cv2.namedWindow("win",    cv2.WINDOW_NORMAL)
        cv2.namedWindow("tf_map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("win",    640, 480)
        cv2.resizeWindow("tf_map", int(MAP_WIDTH_CM*PPM), int(MAP_HEIGHT_CM*PPM))

        # Timers
        self.create_timer(1/30.0, self.vision_cb)
        self.create_timer(0.5,    self.check_and_publish_next)

        # 안내 메시지: 키 ↔ 목적지 매핑
        self.get_logger().info("0:Home, 1:T1, 2:T2, 3:T3, 8:Pick, 9:Drop 키 입력 → A* 경로 생성 및 전송")


    def build_homography(self, corners, ids):
        if ids is None: return None
        img_pts, world_pts = [], []
        for i, mid in enumerate(ids.flatten()):
            if mid in marker_world:
                pts = corners[i][0]
                img_pts.append([float(pts[:,0].mean()), float(pts[:,1].mean())])
                world_pts.append(marker_world[mid])
        if len(img_pts) < 4: return None
        H, _ = cv2.findHomography(np.array(img_pts,np.float32), np.array(world_pts,np.float32))
        return H

    def pixel_to_world(self, H, px, py):
        pt = np.array([px,py,1.0],dtype=np.float32)
        w  = H @ pt; w /= w[2]
        return float(w[0]), float(w[1])

    def vision_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            cv2.waitKey(1)
            return

        # Grayscale and marker detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Update homography
        Hn = self.build_homography(corners, ids)
        if Hn is not None:
            self.cached_H = Hn

        # — marker 5 검출 블록 시작 —
        if ids is not None and self.cached_H is not None and 5 in ids.flatten():
            i5 = list(ids.flatten()).index(5)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.mtx, self.dist)
            R, _    = cv2.Rodrigues(rvecs[i5][0])
            c5      = corners[i5][0]
            px, py  = float(c5[:,0].mean()), float(c5[:,1].mean())
            wx_r, wy_r = self.pixel_to_world(self.cached_H, px, py)
            self.current_pose = (wx_r, wy_r)

            # publish pose
            p = PointStamped()
            p.header.frame_id = 'map'
            p.header.stamp    = self.get_clock().now().to_msg()
            p.point.x, p.point.y, p.point.z = wx_r, wy_r, 0.0
            self.pose_pub.publish(p)

            # publish front
            front3d = np.array([[0.0, self.marker_length, 0.0]], dtype=np.float32)
            camf    = (R @ front3d.T + tvecs[i5][0].reshape(3,1)).T
            imgpts, _ = cv2.projectPoints(camf, np.zeros(3), np.zeros(3),
                                          self.mtx, self.dist)
            fx, fy    = imgpts[0].ravel().astype(int)
            wx_f, wy_f = self.pixel_to_world(self.cached_H, fx, fy)
            f = PointStamped()
            f.header = p.header
            f.point.x, f.point.y, f.point.z = wx_f, wy_f, 0.0
            self.front_pub.publish(f)

            # arrival check & /arrived publish
            if self.active_target:
                dx, dy = self.active_target[0] - wx_r, self.active_target[1] - wy_r
                if math.hypot(dx, dy) <= 2.0:
                    self.get_logger().info("■ 도착 — next 좌표 발행 및 arrived 신호")
                    self.arrived_pub.publish(Empty())
                    self.active_target = None
        # — marker 5 검출 블록 끝 —

        # show camera view
        cv2.imshow("win", frame)

        # bird’s-eye view
        if self.cached_H is not None:
            S      = np.array([[PPM, 0, 0], [0, -PPM, int(MAP_HEIGHT_CM*PPM)], [0, 0, 1]], np.float32)
            tf_map = cv2.warpPerspective(frame, S @ self.cached_H,
                                         (int(MAP_WIDTH_CM*PPM), int(MAP_HEIGHT_CM*PPM)))
            for x, y in self.path_queue:
                px, py = int(x*PPM), int(int(MAP_HEIGHT_CM*PPM)-y*PPM)
                cv2.circle(tf_map, (px, py), 6, (0,255,0), -1)
            if self.active_target:
                tx, ty = self.active_target
                px, py = int(tx*PPM), int(int(MAP_HEIGHT_CM*PPM)-ty*PPM)
                cv2.circle(tf_map, (px, py), 8, (0,0,255), -1)
            cv2.imshow("tf_map", tf_map)

        # key handling: A* path generation only
        key = cv2.waitKey(1) & 0xFF
        if chr(key).isdigit():
            pid = int(chr(key))
            if pid in self.key_to_dest_id and self.current_pose is not None:
                # 터미널에 누른 키와 목적지 이름 출력
                dest_name = self.id_to_name[self.key_to_dest_id[pid]]
                self.get_logger().info(f"키 {pid} → 목적지: {dest_name}")

                # A* 시작 노드 찾기
                start    = min(
                    self.node_coords,
                    key=lambda n: math.hypot(
                        self.current_pose[0] - self.node_coords[n][0],
                        self.current_pose[1] - self.node_coords[n][1]
                    )
                )
                goal_id   = self.key_to_dest_id[pid]
                goal_name = self.id_to_name[goal_id]
                seq       = astar_graph(self.node_coords, self.adj, start, goal_name)

                # path_queue에 좌표 채우기
                self.path_queue = [self.node_coords[n] for n in seq[1:]]
                self.get_logger().info(f"✅ A* 경로: {seq}")
        elif key == ord('q'):
            rclpy.shutdown()


    def check_and_publish_next(self):
        if self.active_target or not self.path_queue:
            return
        x, y = self.path_queue.pop(0)
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = float(x), float(y), 0.0
        self.target_pub.publish(msg)
        self.active_target = (x, y)
        self.get_logger().info(f"📍 좌표 전송: ({x:.1f}, {y:.1f})")

def main():
    rclpy.init()
    node = ArucoNavCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
