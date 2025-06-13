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
    #모터 비활성화
    print("전체 모터를 비활성화합니다.")
    mc.release_all_servos()
    time.sleep(1)
    
    # # # 모터 활성화
    # print("전체 모터를 활성화합니다.")
    # mc.focus_all_servos()
    # time.sleep(1)

if __name__ == '__main__':
    main()
