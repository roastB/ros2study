import os
import time
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle

# JetCobot 연결
mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True
print("[✔] 로봇이 연결되었습니다.")

# 기본 설정
move_speed = 80
home = [87.71, -1.49, 0.7, -1.31, -88.15, 50.18]
angle_grab_bowl_ready = [-90.96, -11.07, -35.36, -38.33, -0.79, -47.02]
angle_wave_1 = [49.04, 26.36, 4.21, 115.57, 3.6, -45.26]
angle_wave_2 = [45.43, 26.36, 47.9, 131.13, -5.97, -46.84]

def wait_until_stable(timeout=3.0):
    start = time.time()
    while mc.is_moving():
        if time.time() - start > timeout:
            print("Timeout: 움직임 완료 확인 실패.")
            break
        time.sleep(0.05)

def shake_motion(repeat=5):
    for idx in range(repeat):
        current = mc.get_angles()
        shake_right = current.copy()
        shake_right[0] = min(shake_right[0] - 20, 168)
        shake_right[5] = max(min(current[5] - (30 * (repeat - idx)), 180), -180)

        mc.send_angles(shake_right, move_speed)
        wait_until_stable()

        # 6번 관절 다시 원위치로 복귀
        mc.send_angles(current, move_speed)
        wait_until_stable()


# [전반적 동작] 전체 물기 털고 + 잔물기 터는 동작 포함 함수
def shake_off_water_motion():
    mc.set_gripper_value(0, move_speed)
    mc.send_angles(angle_grab_bowl_ready, move_speed)
    wait_until_stable()

    for _ in range(3):
        mc.send_angles(angle_wave_1, move_speed)
        wait_until_stable()
        mc.send_angles(angle_wave_2, move_speed+20)
        wait_until_stable()

    print("[...] Shake off water Motion 실행 시작")
    shake_motion()

    mc.send_angles(angle_grab_bowl_ready, move_speed)
    wait_until_stable()

    mc.set_gripper_value(100, move_speed)

# Shake_off_water 동작
shake_off_water_motion()
print("[✔] Shake off water Motion 실행 완료")
