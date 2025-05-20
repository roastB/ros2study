from pymycobot.mycobot280 import MyCobot280

def get_robot():
    # 포트와 통신 속도는 환경에 맞게 조정하세요.
    mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
    mc.thread_lock = True
    print("로봇이 연결되었습니다.")
    return mc
