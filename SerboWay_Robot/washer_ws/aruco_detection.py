import cv2
import numpy as np
import os
import time
import datetime
# from scipy.spatial.transform import Rotation as R

# === ROS2 관련 임포트 ===
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

def live_aruco_detection(ros_node, publisher):
    """
    실시간으로 비디오를 받아 ArUco 마커를 검출하고 3D 포즈를 추정하는 함수
    Args:
        calibration_data: 카메라 캘리브레이션 데이터를 포함한 딕셔너리
        ros_node: rclpy Node 객체
        publisher: ROS2 퍼블리셔 객체
    """
    # camera_matrix = calibration_data['camera_matrix']
    # dist_coeffs = calibration_data['dist_coeffs']

    # 카메라 캘리브레이션 매트릭스 (실제 카메라 캘리브레이션 결과)
    camera_matrix = np.array([
        [965.9166294, 0, 286.74986444],
        [0, 963.60630608, 165.32826283],
        [0, 0, 1]
    ])

    dist_coeffs = np.array([
        [-3.81770715e-01, -4.12378540e-01, -4.33447987e-04, -1.65533463e-04, 1.79885791e+00]
    ])

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    marker_size = 20  # 35mm
    half_size = marker_size / 2
    marker_3d_edges = np.array([
        [-half_size, -half_size, 0],
        [-half_size, half_size, 0],
        [half_size, half_size, 0],
        [half_size, -half_size, 0]
    ], dtype='float32').reshape((4, 1, 3))
    blue_BGR = (255, 0, 0)
    cap = cv2.VideoCapture(0)
 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    time.sleep(2)
    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        corners, ids, rejected = detector.detectMarkers(frame_undistorted)
        if corners:
            for idx, corner in enumerate(corners):
                corner = np.array(corner).reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner
                topRightPoint = (int(topRight[0]), int(topRight[1]))
                topLeftPoint = (int(topLeft[0]), int(topLeft[1]))
                bottomRightPoint = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeftPoint = (int(bottomLeft[0]), int(bottomLeft[1]))
                cv2.circle(frame_undistorted, topLeftPoint, 4, blue_BGR, -1)
                cv2.circle(frame_undistorted, topRightPoint, 4, blue_BGR, -1)
                cv2.circle(frame_undistorted, bottomRightPoint, 4, blue_BGR, -1)
                cv2.circle(frame_undistorted, bottomLeftPoint, 4, blue_BGR, -1)
                ret, rvec, tvec = cv2.solvePnP(
                    marker_3d_edges, 
                    corner, 
                    camera_matrix, 
                    dist_coeffs
                )

                # R_obj_cam, _ = cv2.Rodrigues(rvec)
                # rpy = R.from_matrix(R_obj_cam).as_euler('xyz', degrees=True)

                if ret:
                    x = float(round(tvec[0][0], 2))
                    y = float(round(tvec[1][0], 2))
                    z = float(round(tvec[2][0], 2))
                    rx = float(round(np.rad2deg(rvec[0][0]), 2))
                    ry = float(round(np.rad2deg(rvec[1][0]), 2))
                    rz = float(round(np.rad2deg(rvec[2][0]), 2))
                    pose_estimate = f"{x}, {y}, {z}, {rx}, {ry}, {rz}"
                    print(pose_estimate)

                    text = pose_estimate
                    font = cv2.FONT_HERSHEY_PLAIN
                    font_scale = 1
                    thickness = 1
                    color = (0, 0, 255)
                    cv2.putText(frame_undistorted, text, (10,40), font, font_scale, color, thickness, cv2.LINE_AA)
                    cv2.drawFrameAxes(frame_undistorted, camera_matrix, dist_coeffs,
                                    rvec, tvec, marker_size/2)
        cv2.imshow('ArUco Marker Detection', frame_undistorted)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('a'):
            msg = Float64MultiArray()
            msg.data = [x, y, z, rx, ry, rz]
            publisher.publish(msg)
            ros_node.get_logger().info(f"Published aruco pose: {msg.data}")

        elif key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

def main():
    rclpy.init()
    ros_node = rclpy.create_node('aruco_pose_publisher')
    publisher = ros_node.create_publisher(Float64MultiArray, '/aruco_pose', 10)
    print("Starting ArUco marker detection...")
    live_aruco_detection(ros_node, publisher)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()