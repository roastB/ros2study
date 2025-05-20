import time
from aruco_stackbot.robot import get_robot

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
