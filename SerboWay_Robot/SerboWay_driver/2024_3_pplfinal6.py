#!/usr/bin/env python3
import math
import pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg      import Empty, Int32 
from std_msgs.msg      import Float32, Bool
import heapq
import threading

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
    # original nodes
    11: (44, 31),   # P1
    12: (138, 31),  # P2
    13: (144, 67),  # P3
    14: (42, 71),   # P4
    21: (70, 67),   # T1
    22: (94, 67),   # T2
    23: (118, 67),  # T3
    31: (144, 29),  # Pick
    32: (32,  67),  # Drop
    96: (45,  95),   # Home3cp
    97: (10,  97),   # Home3
    98: (98,  85),   # Home2
    99: (92,  15),  # Home

    # 9 points between P1 (11) and P2 (12)
    101: ( 53.4, 31.0), 102: ( 62.8, 31.0), 103: ( 72.2, 31.0),
    104: ( 81.6, 31.0), 105: ( 91.0, 31.0), 106: (100.4, 31.0),
    107: (109.8, 31.0), 108: (119.2, 31.0), 109: (128.6, 31.0),

    # 9 points between P3 (13) and P4 (14)
    110: (133.8, 67.4), 111: (123.6, 67.8), 112: (113.4, 68.2),
    113: (103.2, 68.6), 114: ( 93.0, 69.0), 115: ( 82.8, 69.4),
    116: ( 72.6, 69.8), 117: ( 62.4, 70.2), 118: ( 52.2, 70.6),

    # 4 points between P2 (12) and P3 (13)
    119: (139.2, 38.2), 120: (140.4, 45.4),
    121: (141.6, 52.6), 122: (142.8, 59.8),

    # 4 points between P1 (11) and P4 (14)
    123: ( 43.6, 39.0), 124: ( 43.2, 47.0),
    125: ( 42.8, 55.0), 126: ( 42.4, 63.0),
}

def build_graph():
    node_coords = {
        'P1':       id_to_coord[11],
        'P2':       id_to_coord[12],
        'P3':       id_to_coord[13],
        'P4':       id_to_coord[14],
        'T1':       id_to_coord[21],
        'T2':       id_to_coord[22],
        'T3':       id_to_coord[23],
        'Pick':     id_to_coord[31],
        'Drop':     id_to_coord[32],
        'Home3cp':     id_to_coord[96],
        'Home3':     id_to_coord[97],
        'Home2':     id_to_coord[98],
        'Home':     id_to_coord[99],
        
        # intermediates P1↔P2
        'P1_P2_1':  id_to_coord[101],
        'P1_P2_2':  id_to_coord[102],
        'P1_P2_3':  id_to_coord[103],
        'P1_P2_4':  id_to_coord[104],
        'P1_P2_5':  id_to_coord[105],
        'P1_P2_6':  id_to_coord[106],
        'P1_P2_7':  id_to_coord[107],
        'P1_P2_8':  id_to_coord[108],
        'P1_P2_9':  id_to_coord[109],
        # intermediates P3↔P4
        'P3_P4_1':  id_to_coord[110],
        'P3_P4_2':  id_to_coord[111],
        'P3_P4_3':  id_to_coord[112],
        'P3_P4_4':  id_to_coord[113],
        'P3_P4_5':  id_to_coord[114],
        'P3_P4_6':  id_to_coord[115],
        'P3_P4_7':  id_to_coord[116],
        'P3_P4_8':  id_to_coord[117],
        'P3_P4_9':  id_to_coord[118],
        # intermediates P2↔P3
        'P2_P3_1':  id_to_coord[119],
        'P2_P3_2':  id_to_coord[120],
        'P2_P3_3':  id_to_coord[121],
        'P2_P3_4':  id_to_coord[122],
        # intermediates P1↔P4
        'P1_P4_1':  id_to_coord[123],
        'P1_P4_2':  id_to_coord[124],
        'P1_P4_3':  id_to_coord[125],
        'P1_P4_4':  id_to_coord[126],
    }
    adj = {
        # original corners
        'P1':       ['P1_P2_1', 'P1_P4_1'],
        'P2':       ['P1_P2_9', 'P2_P3_1', 'Pick'],
        'P3':       ['P2_P3_4', 'P3_P4_1'],
        'P4':       ['P3_P4_9', 'P1_P4_4', 'Drop'],
        'T1': ['P3_P4_7'],  
        'T2': ['P3_P4_5'],  
        'T3': ['P3_P4_3'],  
        'Pick':     ['P2'],
        'Drop':     ['P4'],
        'Home':     ['P1_P2_5'],
        'Home2':    ['P3_P4_5'],
        'Home3':    ['Home3cp'],
        'Home3cp':    ['Home3', 'P3_P4_8'],

        # P1↔P2 intermediates
        'P1_P2_1': ['P1', 'P1_P2_2'],
        'P1_P2_2': ['P1_P2_1', 'P1_P2_3'],
        'P1_P2_3': ['P1_P2_2', 'P1_P2_4'],
        'P1_P2_4': ['P1_P2_3', 'P1_P2_5'],
        'P1_P2_5': ['P1_P2_4', 'P1_P2_6'],
        'P1_P2_6': ['P1_P2_5', 'P1_P2_7'],
        'P1_P2_7': ['P1_P2_6', 'P1_P2_8'],
        'P1_P2_8': ['P1_P2_7', 'P1_P2_9', ],
        'P1_P2_9': ['P1_P2_8', 'P2'],

        # P3↔P4 intermediates
        'P3_P4_1': ['P3',     'P3_P4_2'],
        'P3_P4_2': ['P3_P4_1','P3_P4_3'],
        'P3_P4_3': ['P3_P4_2','P3_P4_4'],
        'P3_P4_4': ['P3_P4_3','P3_P4_5'],
        'P3_P4_5': ['P3_P4_4','P3_P4_6'],
        'P3_P4_6': ['P3_P4_5','P3_P4_7'],
        'P3_P4_7': ['P3_P4_6','P3_P4_8'],
        'P3_P4_8': ['P3_P4_7','P3_P4_9'],
        'P3_P4_9': ['P3_P4_8','P4'],

        # P2↔P3 intermediates
        'P2_P3_1': ['P2',     'P2_P3_2'],
        'P2_P3_2': ['P2_P3_1','P2_P3_3'],
        'P2_P3_3': ['P2_P3_2','P2_P3_4'],
        'P2_P3_4': ['P2_P3_3','P3'],

        # P1↔P4 intermediates
        'P1_P4_1': ['P1',     'P1_P4_2'],
        'P1_P4_2': ['P1_P4_1','P1_P4_3'],
        'P1_P4_3': ['P1_P4_2','P1_P4_4'],
        'P1_P4_4': ['P1_P4_3','P4'],
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
        'Home':99,
        'Home2':98,
        'Home3':97
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

        # 사용할 로봇 마커 ID 및 토픽 suffix 매핑
        self.robot_ids = [5, 6, 7]
        suffix_map = {5: '', 6: '2', 7: '3'}

        # ─── Robot별 Home 좌표 매핑 ───
        self.robot_home = {
            5: id_to_coord[99],   # Home
            6: id_to_coord[98],   # Home2
            7: id_to_coord[97],   # Home3
        }
        self.robot_home_id = {
            5: 99,
            6: 98,
            7: 97,
        }
        # ──────────────────────────────

        # Publishers for each robot
        self.pose_pubs    = {}
        self.front_pubs   = {}
        self.target_pubs  = {}
        self.arrived_pubs = {}
        for rid in self.robot_ids:
            suf = suffix_map[rid]
            self.pose_pubs[rid]    = self.create_publisher(PointStamped, f'/robot_pose{suf}',   10)
            self.front_pubs[rid]   = self.create_publisher(PointStamped, f'/robot_front{suf}',  10)
            self.target_pubs[rid]  = self.create_publisher(PointStamped, f'/target_point{suf}', 10)
            self.arrived_pubs[rid] = self.create_publisher(Empty,        f'/arrived{suf}',     10)
        
        # Topic subscriptions for command & task
        from std_msgs.msg import Int32
        self.create_subscription(Int32, '/pinky_command', self.command_cb, 10)
        self.create_subscription(Int32, '/pinky_task',    self.task_cb,    10)
        # ─── 추가: pinky 퍼블리셔 ───
        self.status_pubs = {}
        self.dist_pubs   = {}
        self.arrive_bool_pubs = {}
        for rid in self.robot_ids:
            suf = suffix_map[rid]
            # 1) 한 번만 발행하는 task ID (Int32)
            self.status_pubs[rid]      = self.create_publisher(Int32,   f'/pinky_status{suf}', 10)
            # 2) 실시간 거리 발행 (Float32)
            self.dist_pubs[rid]        = self.create_publisher(Float32, f'/pinky_dist{suf}',   10)
            # 3) 도착 시 한 번만 발행 (Bool)
            self.arrive_bool_pubs[rid] = self.create_publisher(Bool,    f'/pinky_arrive{suf}', 10)
        # Load camera calibration
        with open(CALIB_PATH, 'rb') as f:
            calib = pickle.load(f)
        self.mtx, self.dist    = calib['camera_matrix'], calib['dist_coeffs']
        self.aruco_dict        = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params            = aruco.DetectorParameters()
        self.marker_length     = MARKER_SIZE_CM
        self.cap               = cv2.VideoCapture(2, cv2.CAP_V4L2)

        # Graph setup
        self.node_coords, self.adj, self.name_to_id, self.id_to_name = build_graph()
        self.current_pose       = {rid: None for rid in self.robot_ids}
        self.prev_pose_for_line = {rid: None for rid in self.robot_ids}
        self.path_queue         = {rid: []   for rid in self.robot_ids}
        self.active_target      = {rid: None for rid in self.robot_ids}

        # 선택된 로봇 ID (None 또는 5,6,7)
        self.selected_robot = None
        # Homography 캐싱용 변수 초기화
        self.cached_H = None
        # 키→목적지 ID 매핑
        self.key_to_dest_id = {1:21, 2:22, 3:23, 8:31, 9:32, 0:99}

        # Visualization windows
        cv2.namedWindow("win",    cv2.WINDOW_NORMAL)
        cv2.namedWindow("tf_map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("win",    640, 480)
        cv2.resizeWindow("tf_map", int(MAP_WIDTH_CM*PPM), int(MAP_HEIGHT_CM*PPM))

        # Timers
        self.create_timer(1/30.0, self.vision_cb)

        # 안내 메시지
        self.get_logger().info(
            "Use /pinky_command to select robot (5/6/7) and /pinky_task to send target (0/1/2/3/8/9)"
        )


    # Utility: calculate path length
    def path_length(self, path):
        total = 0.0
        for u, v in zip(path, path[1:]):
            x1, y1 = self.node_coords[u]
            x2, y2 = self.node_coords[v]
            total += math.hypot(x2-x1, y2-y1)
        return total

    # Utility: enumerate all simple paths
    def dfs_all_paths(self, src, dst, visited=None):
        if visited is None:
            visited = [src]
        if src == dst:
            yield visited.copy()
        for nbr in self.adj[src]:
            if nbr in visited:
                continue
            visited.append(nbr)
            yield from self.dfs_all_paths(nbr, dst, visited)
            visited.pop()

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
    
    def command_cb(self, msg: Int32):
        if msg.data in self.robot_ids:
            self.selected_robot = msg.data
            self.get_logger().info(f"로봇 {self.selected_robot} 선택됨 (via pinky_command)")
        else:
            self.get_logger().warn(f"알 수 없는 로봇 ID: {msg.data}")

    def task_cb(self, msg: Int32):
        pid = msg.data
        rid = self.selected_robot
        if rid not in self.robot_ids:
            self.get_logger().warn("먼저 /pinky_command로 로봇을 선택하세요")
            return

        # 이전 활성 목표 취소
        self.active_target[rid] = None
        self.path_queue[rid].clear()

        # goal_id 결정
        if pid == 0:
            goal_id = self.robot_home_id[rid]
        else:
            goal_id = self.key_to_dest_id.get(pid)
            if goal_id is None:
                self.get_logger().warn(f"알 수 없는 목적지 ID: {pid}")
                return

        # 현재 위치 없으면 무시
        if self.current_pose[rid] is None:
            self.get_logger().warn("현재 로봇 위치 정보가 없습니다")
            return

        # A* 경로 탐색
        start = min(
            [n for n in self.node_coords if n not in ('T1','T2','T3')],
            key=lambda n: heuristic(self.current_pose[rid], self.node_coords[n])
        )
        goal_name = self.id_to_name[goal_id]
        all_paths = list(self.dfs_all_paths(start, goal_name))
        if not all_paths:
            self.get_logger().warn(f"⚠️ 로봇{rid} 경로 없음: {start} → {goal_name}")
            return

        # 최단 경로 선택 및 필터링 (기존과 동일)
        candidates = [(p, self.path_length(p)) for p in all_paths]
        best_path, best_dist = min(candidates, key=lambda x: x[1])
        self.get_logger().info(f"로봇{rid} 최단 경로 {best_path} (거리 {best_dist:.1f}cm)")

        corner = {'P1','P2','P3','P4'}
        keyn   = {'Home','Home2','Home3','Pick','Drop','T1','T2','T3'}
        filtered = []
        if best_path[0] not in keyn:
            filtered.append(best_path[0])
        for i in range(1, len(best_path)):
            node, prev = best_path[i], best_path[i-1]
            if node in corner|keyn:
                if prev not in filtered:
                    filtered.append(prev)
                filtered.append(node)

        # path_queue 세팅 후 첫 좌표 발행
        self.path_queue[rid] = [ self.node_coords[n] for n in filtered ]
        self.check_and_publish_next(rid)
        # ─── 추가: 한 번만 pinky_status 발행 ───
        status_msg = Int32()
        status_msg.data = pid
        self.status_pubs[rid].publish(status_msg)
        # ───────────────────────────────────────

    def vision_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            cv2.waitKey(1)
            return

        # Grayscale & detect markers
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.params
        )
        aruco.drawDetectedMarkers(frame, corners, ids)


        # Update homography if possible
        Hn = self.build_homography(corners, ids)
        if Hn is not None:
            self.cached_H = Hn

        # 마커 5 잡혔을 때: 화살표 + 퍼블리시 + 도착 체크
        # 다중-로봇 처리: 마커 ID 5, 6, 7
        if ids is not None and self.cached_H is not None:
            # rvecs, tvecs 한 번만 계산
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.mtx, self.dist
            )
            for i, mid in enumerate(ids.flatten()):
                if mid not in self.robot_ids:
                    continue

                # 1) 픽셀 → 월드 좌표계 변환
                pts = corners[i][0]
                px, py = float(pts[:,0].mean()), float(pts[:,1].mean())
                wx_r, wy_r = self.pixel_to_world(self.cached_H, px, py)
                self.current_pose[mid] = (wx_r, wy_r)

                # 2) 방향 화살표(arrow) 그리기
                R, _ = cv2.Rodrigues(rvecs[i][0])
                front3d = np.array([[0.0, self.marker_length, 0.0]], dtype=np.float32)
                camf    = (R @ front3d.T + tvecs[i][0].reshape(3,1)).T
                imgpts, _ = cv2.projectPoints(
                    camf, np.zeros(3), np.zeros(3), self.mtx, self.dist
                )
                fx, fy = imgpts[0].ravel().astype(int)
                cv2.arrowedLine(
                    frame,
                    (int(px), int(py)),
                    (fx, fy),
                    (255, 0, 0),
                    2,
                    tipLength=0.2
                )

                # 3) /robot_pose 퍼블리시
                p = PointStamped()
                p.header.frame_id = 'map'
                p.header.stamp    = self.get_clock().now().to_msg()
                p.point.x, p.point.y, p.point.z = wx_r, wy_r, 0.0
                self.pose_pubs[mid].publish(p)

                # 4) /robot_front 퍼블리시
                wx_f, wy_f = self.pixel_to_world(self.cached_H, fx, fy)
                f = PointStamped()
                f.header = p.header
                f.point.x, f.point.y, f.point.z = wx_f, wy_f, 0.0
                self.front_pubs[mid].publish(f)
                # ─── 추가: pinky_dist 발행 ───
                pick_x, pick_y = self.node_coords['Pick']
                dist = math.hypot(wx_r - pick_x, wy_r - pick_y)
                dist_msg = Float32()
                dist_msg.data = float(dist)
                self.dist_pubs[mid].publish(dist_msg)
                # ──────────────────────────────
                # 5) 도착 판정 및 /arrived 퍼블리시
                at = self.active_target[mid]
                if at is not None:
                    dx, dy = at[0] - wx_r, at[1] - wy_r
                    if math.hypot(dx, dy) <= 2.0:
                        self.get_logger().info(f"■ 로봇{mid} 도착")
                        self.arrived_pubs[mid].publish(Empty())
                        # 2) 추가: Bool arrived
                        bool_msg = Bool()
                        bool_msg.data = True
                        self.arrive_bool_pubs[mid].publish(bool_msg)
                        self.active_target[mid] = None
                        threading.Timer(1.0, lambda m=mid: self.check_and_publish_next(m)).start()


        # Show camera image
        cv2.imshow("win", frame)

        # Bird’s-eye view 그리기
        if self.cached_H is not None:
            S = np.array([[PPM,0,0],[0,-PPM,int(MAP_HEIGHT_CM*PPM)],[0,0,1]], np.float32)
            tf_map = cv2.warpPerspective(frame, S @ self.cached_H,
                                        (int(MAP_WIDTH_CM*PPM), int(MAP_HEIGHT_CM*PPM)))
            
            # ▶ 각 로봇별 path_queue 그리기
            for rid in self.robot_ids:
                for x, y in self.path_queue[rid]:
                    cx = int(x * PPM)
                    cy = int(MAP_HEIGHT_CM * PPM - y * PPM)
                    cv2.circle(tf_map, (cx, cy), 6, (0,255,0), -1)
            
            # ▶ 각 로봇별 active_target 그리기
            for rid in self.robot_ids:
                at = self.active_target[rid]
                if at is not None:
                    tx, ty = at
                    cx = int(tx * PPM)
                    cy = int(MAP_HEIGHT_CM * PPM - ty * PPM)
                    cv2.circle(tf_map, (cx, cy), 8, (0,0,255), -1)

            # ▶ 로봇↔목표 선분 (발행 시점 위치)
            for rid in self.robot_ids:
                prev = self.prev_pose_for_line[rid]
                at   = self.active_target[rid]
                if prev and at:
                    rpx = int(prev[0] * PPM)
                    rpy = int(MAP_HEIGHT_CM * PPM - prev[1] * PPM)
                    ntx = int(at[0]  * PPM)
                    nty = int(MAP_HEIGHT_CM * PPM - at[1] * PPM)
                    cv2.line(tf_map, (rpx, rpy), (ntx, nty), (255, 0, 0), 2)

            cv2.imshow("tf_map", tf_map)
        
        cv2.waitKey(1)

    # per-robot next 발행 함수: rid 인자를 받도록 수정
    def check_and_publish_next(self, rid):
        if self.active_target[rid] or not self.path_queue[rid]:
            return

        # 이전 pose 저장
        self.prev_pose_for_line[rid] = self.current_pose[rid]

        # 다음 좌표 꺼내기
        x, y = self.path_queue[rid].pop(0)

        # 퍼블리시
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = float(x), float(y), 0.0
        self.target_pubs[rid].publish(msg)

        # active_target 설정
        self.active_target[rid] = (x, y)
        self.get_logger().info(f"로봇{rid} 📍 좌표 전송: ({x:.1f}, {y:.1f})")


def main():
    rclpy.init()
    node = ArucoNavCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()