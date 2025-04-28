import cv2
import socket
import pickle
import struct
import time
import numpy as np
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord
import os

# MyCobot 초기화
mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True
print("로봇이 연결되었습니다.")

# 이동할 위치 리스트
pose_list = [
    [-90.96, -11.07, -35.36, -38.33, -0.79, -47.02],
    [-74.97, -6.15, -29.53, -51.67, -5.62, -37.96],
    [-64.33, -7.29, -29.53, -54.31, -10.37, -39.02],
    [-98.34, -2.28, -29.88, -48.95, 3.86, -39.19],
    [-81.56, -4.39, -29.88, -52.38, -3.42, -39.19],
    [-54.05, 5.71, -29.88, -64.59, -13.35, -39.37],
    [-104.41, -2.1, -29.88, -49.39, 9.49, -38.93],
    [-88.85, 0.87, -29.88, -52.2, 1.58, -38.84],
    [-12.56, -8.7, -29.88, -84.37, -23.73, -36.12],
    [-106.69, -0.17, -14.32, -63.1, 8.17, -35.41],
    [-78.92, 18.8, -14.23, -76.11, -3.69, -35.41],
]

# 체크보드 패턴 크기 설정
CHECKERBOARD = (9, 6)  # 9x6 내외 코너 수

# 3D 세계 좌표 (0,0,0부터 시작하여 1단위 크기씩 증가하는 좌표)
obj_p = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
obj_p[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# 서버 IP와 포트 설정
server_ip = '192.168.0.167'  # 서버 IP
server_port = 8889  # 포트 번호

# 소켓 설정
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((server_ip, server_port))
server_socket.listen(5)
print(f"[TCP 서버] {server_ip}:{server_port} 대기 중...")

# 클라이언트 연결 대기
client_socket, client_address = server_socket.accept()
print(f"[TCP 서버] 클라이언트 연결됨: {client_address}")

# 카메라 캡처 (JetCobot의 카메라)
cap = cv2.VideoCapture(0)  # JetCobot에서 연결된 카메라 사용 (카메라 장치 번호에 맞게 설정)

if not cap.isOpened():
    print("[!] 카메라를 열 수 없습니다.")
    exit()

# 캘리브레이션을 위한 3D 및 2D 점 리스트
obj_points = []  # 3D 점
img_points = []  # 2D 점

# 이미지를 저장할 디렉토리 설정
save_dir = "/home/jetcobot/sw_ws/data/calibration_images"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# 로봇 이동 및 캡처 후 전송
for idx, pose in enumerate(pose_list):
    print(f"[{idx+1}/{len(pose_list)}] 위치로 이동 중: {pose}")
    
    # 로봇 이동 (send_angles로 위치 이동)
    mc.send_angles(pose, 50)  # 50은 속도
    mc.set_gripper_value(100, 50)  # 그리퍼 열기
    time.sleep(2)  # 움직임이 완료될 때까지 대기

    # 카메라에서 프레임 캡처
    ret, frame = cap.read()
    if not ret:
        print("[!] 프레임 캡처 실패.")
        break

    # 체크보드 코너 찾기
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        # 3D 좌표와 2D 좌표 추가
        obj_points.append(obj_p)
        img_points.append(corners)

        # 코너에 원 그리기
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)
        print(f"[TCP 서버] 위치 {pose}에서 체크보드 코너 찾음.")
    else:
        print(f"[TCP 서버] 위치 {pose}에서 체크보드 코너를 찾을 수 없음.")

    # 프레임 직렬화
    data = pickle.dumps(frame)
    message = struct.pack(">L", len(data)) + data

    # 클라이언트에게 전송
    client_socket.sendall(message)
    print(f"[TCP 서버] 위치 {pose}에서 프레임 전송 완료.")

    # 프레임 저장
    image_filename = os.path.join(save_dir, f"frame_{idx+1}.png")
    cv2.imwrite(image_filename, frame)
    print(f"[TCP 서버] 위치 {pose}에서 이미지 저장 완료: {image_filename}")

    # 잠시 대기 (클라이언트가 프레임을 받을 수 있도록)
    time.sleep(1)

# 카메라 캘리브레이션
if len(obj_points) > 0 and len(img_points) > 0:
    print("[TCP 서버] 카메라 캘리브레이션 중...")

    # 카메라 캘리브레이션
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    
    if ret:
        print("[TCP 서버] 캘리브레이션 성공!")
        print("카메라 행렬:\n", camera_matrix)
        print("왜곡 계수:\n", dist_coeffs)
    else:
        print("[TCP 서버] 캘리브레이션 실패!")

# 자원 해제
cap.release()
client_socket.close()
server_socket.close()
cv2.destroyAllWindows()
