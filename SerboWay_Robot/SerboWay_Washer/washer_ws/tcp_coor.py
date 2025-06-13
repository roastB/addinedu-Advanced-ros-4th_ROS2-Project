import time
from robot import get_robot

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

def main():
    rclpy.init()
    ros_node = rclpy.create_node('tcp_pose_publisher')
    publisher = ros_node.create_publisher(Float64MultiArray, '/tcp_pose', 10)

    mc = get_robot()

    print("p 입력 후 엔터를 누르면 현재 TCP pose를 publish합니다. (q 입력 후 엔터: 종료)")

    try:
        while rclpy.ok():
            user_input = input("p: publish, q: quit > ").strip().lower()
            if user_input == 'q':
                print("종료합니다.")
                break
            elif user_input == 'p':
                current_coords = mc.get_coords()
                print("현재 좌표:", current_coords)

                x = float(round(current_coords[0],2))
                y = float(round(current_coords[1],2))
                z = float(round(current_coords[2],2))
                rx = float(round(current_coords[3],2))
                ry = float(round(current_coords[4],2))
                rz = float(round(current_coords[5],2))

                msg = Float64MultiArray()
                msg.data = [x, y, z, rx, ry, rz]
                publisher.publish(msg)
                ros_node.get_logger().info(f"Published tcp pose: {msg.data}")
            else:
                print("p 또는 q만 입력하세요.")

    except KeyboardInterrupt:
        print("사용자 인터럽트로 종료합니다.")

    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
