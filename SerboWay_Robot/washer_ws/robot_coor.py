import time
from pymycobot.mycobot280 import MyCobot280

def get_robot():
    # 포트와 통신 속도는 환경에 맞게 조정하세요.
    mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
    mc.thread_lock = True
    print("로봇이 연결되었습니다.")
    return mc

def main():
    mc = get_robot()

    current_coords = mc.get_coords()
    print("현재 좌표:", current_coords)

    # matrix 계산 위치 고정
    home_coords = [150.0, -80.0, 280.0, -175.0, -6.0, -45.0]
    mc.send_coords(home_coords, 30, 0)
    time.sleep(2)

    # # 그리퍼 열기
    # mc.set_gripper_value(100, 50)
    # time.sleep(1)

    # # 카트 위에
    # ingredient_coords = [179.8, -96.8, 200.9, -172.01, -0.76, -47.67]
    # mc.send_coords(ingredient_coords, 30, 0)
    # time.sleep(2)

    # # 카트
    # ingredient_coords = [179.8, -96.8, 153.9, -172.01, -0.76, -47.67]
    # mc.send_coords(ingredient_coords, 30, 0)
    # time.sleep(2)

    # # 그리퍼 닫기
    # mc.set_gripper_value(0, 50)
    # time.sleep(1)

    # # 카트 위에
    # ingredient_coords = [179.8, -96.8, 200.9, -172.01, -0.76, -47.67]
    # mc.send_coords(ingredient_coords, 30, 0)
    # time.sleep(2)

    # # 설거지통 위에
    # ingredient_coords = [35.2, -148.7, 255.5, 174.16, 1.95, -133.48]
    # mc.send_coords(ingredient_coords, 30, 0)
    # time.sleep(2)

    # # 설거지통
    # ingredient_coords = [30.9, -147.4, 169.9, 172.85, -0.91, -133.68]
    # mc.send_coords(ingredient_coords, 30, 0)
    # time.sleep(2)

    # # 그리퍼 열기
    # mc.set_gripper_value(100, 50)
    # time.sleep(1)

    # # 설거지통 위에
    # ingredient_coords = [35.2, -148.7, 255.5, 174.16, 1.95, -133.48]
    # mc.send_coords(ingredient_coords, 30, 0)
    # time.sleep(2)

    # mc.send_coords(home_coords, 30, 0)
    # time.sleep(2)


    # mc.send_coords(home_coords, 30, 0)
    # time.sleep(2)


if __name__ == '__main__':
    main()


    # # 1. 먼저 Z축을 낮추기
    # work_coords = current_coords.copy()
    # work_coords[2] -= 50 
    # print(f"Z축을 {work_coords[2]}로 내립니다.")
    # mc.send_coords(work_coords, 30, 0)
    # time.sleep(2)

    # # 2. X 좌표 이동
    # x_coords = work_coords.copy()
    # x_coords[0] += 20 
    # print(f"X 좌표를 {x_coords[0]}로 이동합니다.")
    # mc.send_coords(x_coords, 30, 0)
    # time.sleep(2)

    # # 3. Y 좌표 이동
    # y_coords = x_coords.copy()
    # y_coords[1] -= 20 
    # print(f"Y 좌표를 {y_coords[1]}로 이동합니다.")
    # mc.send_coords(y_coords, 30, 0)
    # time.sleep(2)

    # # 5. 초기 위치로 복귀
    # print("초기 위치로 복귀합니다.")
    # mc.send_angles([0, 0, 0, 0, 0, 0], 135)
    # time.sleep(3)
    # print("초기 위치 복귀 완료")