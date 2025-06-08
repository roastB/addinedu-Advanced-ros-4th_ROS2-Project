# import cv2
# import numpy as np
# import os
# import time
# import datetime
# import pickle

# def live_ingredient_detection(calibration_data):
#     # camera_matrix = calibration_data['camera_matrix']
#     # dist_coeffs = calibration_data['dist_coeffs']

#     # print(camera_matrix)
#     # print(dist_coeffs)

#     # 카메라 캘리브레이션 매트릭스 (실제 카메라 캘리브레이션 결과)
#     camera_matrix = np.array([
#         [965.9166294, 0, 286.74986444],
#         [0, 963.60630608, 165.32826283],
#         [0, 0, 1]
#     ])

#     dist_coeffs = np.array([
#         [-3.81770715e-01, -4.12378540e-01, -4.33447987e-04, -1.65533463e-04, 1.79885791e+00]
#     ])
    
#     # 카메라 설정
#     cap = cv2.VideoCapture('/dev/video0')
    
#     # 카메라 초기화 대기
#     time.sleep(2)
    
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("Failed to grab frame")
#             break
            
#         # 이미지 왜곡 보정
#         frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        
#         # --- 사각형 검출 시작 ---
#         hsv = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2HSV)
#         lower_blue = np.array([100, 150, 50])
#         upper_blue = np.array([140, 255, 255])

#         lower_gray = np.array([0, 0, 180])
#         upper_gray = np.array([180, 30, 230])

#         lower_green = np.array([35, 50, 50])
#         upper_green = np.array([85, 255, 255])

#         lower_yellow = np.array([15, 150, 20])
#         upper_yellow = np.array([35, 255, 255])

#         lower_black = np.array([0, 0, 0])
#         upper_black = np.array([180, 255, 100])

#         hsv_mask = cv2.inRange(hsv, lower_black, upper_black)
#         cv2.imshow('hsv', hsv_mask)

#         # 노이즈 제거 (옵션)
#         kernel = np.ones((7, 7), np.uint8)
#         mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_CLOSE, kernel)
        
#         cv2.imshow('morphlogy Detection', mask)

#         # 외곽선 검출
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         for cnt in contours:
#             epsilon = 0.02 * cv2.arcLength(cnt, True)
#             approx = cv2.approxPolyDP(cnt, epsilon, True)
#             # 사각형(꼭짓점 4개) & 면적 필터링
#             if len(approx) == 4 and cv2.contourArea(approx) > 1000:
#                 # 사각형 그리기
#                 cv2.polylines(frame_undistorted, [approx], True, (255, 0, 0), 3)
#                 # 중심점 계산
#                 pts = approx.reshape(4, 2)
#                 cx = int(np.mean(pts[:, 0]))
#                 cy = int(np.mean(pts[:, 1]))
#                 # 중심점 표시
#                 cv2.circle(frame_undistorted, (cx, cy), 5, (0, 0, 255), -1)
#                 # 중심점 좌표 출력
#                 print(f"사각형 중심점: ({cx}, {cy})")
        
#         # --- 색상 사각형 검출 끝 ---

#         # 프레임 표시
#         cv2.imshow('Rectangle Detection', frame_undistorted)
        
#         # 'q' 키를 누르면 종료
#         key = cv2.waitKey(1) & 0xFF
#         if key == ord('q'):
#             break

#     # 리소스 해제
#     cap.release()
#     cv2.destroyAllWindows()

# def main():
#     # 캘리브레이션 데이터 로드

#     print("Starting Rectangle Detection...")
#     live_ingredient_detection(calibration_data)

# if __name__ == "__main__":
#     main()

import cv2
import numpy as np
import os
import time
import datetime
import pickle
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

def live_ingredient_detection(ros_node, publisher):
    # camera_matrix = calibration_data['camera_matrix']
    # dist_coeffs = calibration_data['dist_coeffs']

    # print(camera_matrix)
    # print(dist_coeffs)

    # 카메라 캘리브레이션 매트릭스 (실제 카메라 캘리브레이션 결과)
    camera_matrix = np.array([
        [965.9166294, 0, 286.74986444],
        [0, 963.60630608, 165.32826283],
        [0, 0, 1]
    ])

    cx = 286.74986444
    cy = 165.32826283
    fx = 965.9166294
    fy = 963.60630608
    d = 150
    depth_scale = 1


    dist_coeffs = np.array([
        [-3.81770715e-01, -4.12378540e-01, -4.33447987e-04, -1.65533463e-04, 1.79885791e+00]
    ])
    
    # 카메라 설정
    cap = cv2.VideoCapture('/dev/video0')
    
    # 카메라 초기화 대기
    time.sleep(2)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
            
        # 이미지 왜곡 보정
        frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        
        # --- 사각형 검출 시작 ---
        hsv = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        lower_gray = np.array([0, 0, 180])
        upper_gray = np.array([180, 30, 230])

        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])

        lower_yellow = np.array([0, 60, 20])
        upper_yellow = np.array([50, 220, 255])

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        lower_plate = np.array([85, 5, 148])
        upper_plate = np.array([120, 110, 250])

        hsv_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        cv2.imshow('hsv', hsv_mask)

        # 노이즈 제거 (옵션)
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_CLOSE, kernel)
        cv2.imshow('image', frame_undistorted)
        cv2.imshow('morphlogy Detection', mask)

        frame_rec = frame_undistorted.copy()

        # 외곽선 검출
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            rec_area = cv2.contourArea(approx)
            # 사각형(꼭짓점 4개) & 면적 필터링
            if len(approx) == 4 and rec_area > 1000:
                # 사각형 그리기
                cv2.polylines(frame_undistorted, [approx], True, (255, 0, 0), 3)
                # 중심점 계산
                pts = approx.reshape(4, 2)
                u = int(np.mean(pts[:, 0]))
                v = int(np.mean(pts[:, 1]))

                Z = d / depth_scale
                # Z = 50
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy

                # 중심점 표시
                cv2.circle(frame_undistorted, (u, v), 5, (0, 0, 255), -1)
                # 중심점 좌표 출력
                print(f"사각형 중심점: ({u}, {v})")
                print(rec_area)

        # 외곽선 검출
        _, mask = cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            rec_area = cv2.contourArea(approx)
            # 사각형(꼭짓점 4개) & 면적 필터링
            if len(approx) == 4 and rec_area > 1000 and rec_area < 100000:
                # 사각형 그리기
                cv2.polylines(frame_rec, [approx], True, (255, 0, 0), 3)
                # 중심점 계산
                pts = approx.reshape(4, 2)
                u = int(np.mean(pts[:, 0]))
                v = int(np.mean(pts[:, 1]))

                Z = d / depth_scale
                # Z = 50
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy

                # 중심점 표시
                cv2.circle(frame_rec, (u, v), 5, (0, 0, 255), -1)
                # 중심점 좌표 출력
                print(f"사각형 중심점: ({u}, {v})")
                print(rec_area)

                
        
        # --- 색상 사각형 검출 끝 ---

        # 프레임 표시
        cv2.imshow('Rectangle Detection', frame_undistorted)
        # 프레임 표시
        cv2.imshow('Rectangle', frame_rec)
        
        key = cv2.waitKey(1) & 0xFF

        if key == ord('a'):
            msg = Float64MultiArray()
            msg.data = [X,Y,Z]
            publisher.publish(msg)
            ros_node.get_logger().info(f"Published rectagle pose: {msg.data}")

        elif key == ord('q'):
            break

    # 리소스 해제
    cap.release()
    cv2.destroyAllWindows()

def main():
    # 캘리브레이션 데이터 로드
    rclpy.init()
    ros_node = rclpy.create_node('color_pose_publisher')
    publisher = ros_node.create_publisher(Float64MultiArray, '/color_pose', 10)
    print("Starting rectagle detection...")
    live_ingredient_detection(ros_node, publisher)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
