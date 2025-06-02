import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from pymycobot.mycobot280 import MyCobot280

def get_robot():
    # 포트와 통신 속도는 환경에 맞게 조정하세요.
    mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
    mc.thread_lock = True
    print("로봇이 연결되었습니다.")
    return mc

# 1. 데이터 입력 (각 행: [x, y, z, roll, pitch, yaw])
A_data = np.array([
    [-24.98, -4.28, 168.53, 124.11, 117.99, 2.32],
    [-21.73, 31.42, 159.41, 116.13, 115.17, 18.13],
    [-2.25, 14.77, 169.92, 123.04, 119.67, -0.92],
    [25.06, 1.6, 169.74, -130.44, -124.76, 7.99],
    [38.13, -10.16, 165.12, -124.2, -125.77, 22.58],
    [16.42, 26.61, 165.08, 116.98, 117.52, -7.94],
    [37.21, 30.29, 157.22, 113.76, 116.13, -23.11]
])
B_data = np.array([
    [217.5, -79.3, 117.3, -174.97, 0.65, -45.74],
    [182.2, -73.1, 116.8, -177.74, -1.98, -46.35],
    [200.7, -92.3, 116.2, -177.53, -2.18, -47.14],
    [208.4, -125.0, 115.5, -173.61, -2.55, -45.81],
    [218.2, -137.5, 116.6, -174.32, -3.1, -48.59],
    [186.0, -114.4, 115.5, -176.5, -1.9, -45.78],
    [181.5, -135.6, 118.9, -177.0, -1.74, -44.73]
])
# 2. 위치만 추출
A_pos = A_data[:, :3]
B_pos = B_data[:, :3]

# 3. Kabsch 알고리즘 (Rigid transform: 회전 R, 이동 t)
def rigid_transform_3D(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R_mat = Vt.T @ U.T
    if np.linalg.det(R_mat) < 0:
        Vt[2, :] *= -1
        R_mat = Vt.T @ U.T
    t = centroid_B - R_mat @ centroid_A
    return R_mat, t

R_pos, t_pos = rigid_transform_3D(A_pos, B_pos)

# 4. 오일러 각 → 회전행렬 변환
def euler_to_matrix(angles_deg):
    # angles_deg: [roll, pitch, yaw] (각도, 단위: degree)
    return R.from_euler('xyz', angles_deg, degrees=True).as_matrix()

A_rotmats = np.array([euler_to_matrix(a[3:]) for a in A_data])
B_rotmats = np.array([euler_to_matrix(b[3:]) for b in B_data])

# 5. 상대 회전행렬 구하기 (B = R_ori * A)
relative_rotmats = [B_rotmats[i] @ A_rotmats[i].T for i in range(len(A_rotmats))]
# 평균 회전행렬 구하기 (scipy의 Rotation 평균 사용)
R_ori = R.from_matrix(relative_rotmats).mean().as_matrix()

# 6. 변환 함수
def transform_pose_A_to_B(pos_A, rotmat_A):
    pos_B = R_pos @ pos_A + t_pos
    rotmat_B = R_ori @ rotmat_A
    return pos_B, rotmat_B

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'aruco_pose',  # 실제 토픽명으로 변경
            self.listener_callback,
            10)
        # 필요한 데이터 미리 계산
        self.A_rotmats = A_rotmats
        self.R_pos = R_pos
        self.t_pos = t_pos
        self.R_ori = R_ori
        self.gripper_open_size = 100

    def listener_callback(self, msg):
        
        print(0)
        if len(msg.data) == 0:
            self.get_logger().info("No marker detected")
            return
        print(1)
        
        mc = get_robot()

        pose6d = msg.data
        pos_A = np.array([pose6d[0], pose6d[1], pose6d[2]])
        euler_A = np.array([pose6d[3], pose6d[4], pose6d[5]])
        rotmat_A = euler_to_matrix(euler_A)
        
        pos_B, rotmat_B = transform_pose_A_to_B(pos_A, rotmat_A)
        euler_B = R.from_matrix(rotmat_B).as_euler('xyz', degrees=True)
        B_rot = B_data[0, 3:]
        print("=== 변환 결과 ===")
        print("입력 위치 (A):", pos_A)
        print("변환된 위치 (B):", pos_B)
        print("변환된 오일러 각(roll, pitch, yaw):", euler_B)

        # 그리퍼 열기
        mc.set_gripper_value(self.gripper_open_size, 50)
        time.sleep(1)

        print("지정한 좌표로 이동합니다.")
        custom_coords = [pos_B[0], pos_B[1], pos_B[2], B_rot[0], B_rot[1], B_rot[2]]
        #mc.send_coords(custom_coords, 30, 1)
        #time.sleep(2)
        # custom_coords_10 = [pos_B[0], pos_B[1], pos_B[2] , euler_B[0], euler_B[1], euler_B[2]]
        # mc.send_coords(custom_coords_10, 30, 1)
        # time.sleep(2)
        #print("지정한 좌표:", custom_coords)

        # 1. 먼저 Z축을 낮추기
        work_coords = custom_coords.copy()
        work_coords[2] += 10 
        #print(f"Z축을 {work_coords[2]}로 내립니다.")
        mc.send_coords(work_coords, 30, 0)
        time.sleep(2)
        
        # 그리퍼 닫기
        mc.set_gripper_value(0, 50)
        time.sleep(1)

        print("지정한 각도로 이동합니다.")
        custom_angles = [0, 0, 0, 0, 0, -45]
        mc.send_angles(custom_angles, 30)
        time.sleep(3)
        print("지정한 각도:", custom_angles)


        # for marker in msg.data: 
        #     # marker = msg.markers[0]
        #     pose6d = marker.pose_of_6d
        #     if len(pose6d) < 6:
        #         self.get_logger().warn('pose_of_6d 길이가 6보다 작습니다.')
        #         continue
        #     print(2)
        #     pos_A = np.array([pose6d[0], pose6d[1], pose6d[2]])
        #     euler_A = np.array([pose6d[3], pose6d[4], pose6d[5]])
        #     rotmat_A = euler_to_matrix(euler_A)
            
        #     pos_B, rotmat_B = transform_pose_A_to_B(pos_A, rotmat_A)
        #     euler_B = R.from_matrix(rotmat_B).as_euler('xyz', degrees=True)
        #     print("=== 변환 결과 ===")
        #     print("입력 위치 (A):", pos_A)
        #     print("변환된 위치 (B):", pos_B)
        #     print("변환된 오일러 각(roll, pitch, yaw):", euler_B)

        #     # 그리퍼 열기
        #     mc.set_gripper_value(self.gripper_open_size, 50)
        #     time.sleep(1)

        #     print("지정한 좌표로 이동합니다.")
        #     custom_coords = [pos_B[0], pos_B[1], pos_B[2], euler_B[0], euler_B[1], euler_B[2]]
        #     mc.send_coords(custom_coords, 30, 1)
        #     time.sleep(2)
        #     # custom_coords_10 = [pos_B[0], pos_B[1], pos_B[2] , euler_B[0], euler_B[1], euler_B[2]]
        #     # mc.send_coords(custom_coords_10, 30, 1)
        #     # time.sleep(2)
        #     print("지정한 좌표:", custom_coords)
            
        #     # 그리퍼 닫기
        #     mc.set_gripper_value(0, 50)
        #     time.sleep(1)

        #     print("지정한 각도로 이동합니다.")
        #     custom_angles = [0, 0, 0, 0, 0, 135]
        #     mc.send_angles(custom_angles, 30)
        #     time.sleep(3)
        #     print("지정한 각도:", custom_angles)


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
