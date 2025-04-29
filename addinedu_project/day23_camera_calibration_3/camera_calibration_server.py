#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
로봇 카메라 캘리브레이션 스크립트
마이코봇(MyCobot)을 사용하여 다양한 위치에서 체크보드 이미지를 캡처하고
카메라 캘리브레이션을 수행하는 프로그램
"""

import os
import time
import socket
import pickle
import struct
import numpy as np
import cv2
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

# 상수 정의
ROBOT_PORT = '/dev/ttyJETCOBOT'
ROBOT_BAUDRATE = 1000000
SERVER_IP = '192.168.0.167'
SERVER_PORT = 8889
CAMERA_ID = 0
CHECKERBOARD = (9, 6)  # 체크보드 패턴 크기 (가로, 세로)
SAVE_DIR = "/home/jetcobot/sw_ws/data/calibration_images"
ROBOT_MOVE_SPEED = 50
ROBOT_POSITION_THRESHOLD = 2.0
ROBOT_TIMEOUT = 10
CAMERA_BUFFER_FRAMES = 5


class RobotCameraCalibration:
    """로봇 카메라 캘리브레이션을 위한 클래스"""
    
    def __init__(self):
        """클래스 초기화"""
        # 마이코봇 초기화
        self.robot = self._init_robot()
        
        # 카메라 초기화
        self.cap = self._init_camera()
        
        # 서버 소켓 설정 및 클라이언트 연결
        self.server_socket, self.client_socket = self._init_network()
        
        # 캘리브레이션 데이터 저장용 리스트
        self.obj_points = []
        self.img_points = []
        
        # 이미지 저장 디렉토리 생성
        os.makedirs(SAVE_DIR, exist_ok=True)
        
        # 3D 세계 좌표 생성
        self.obj_p = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        self.obj_p[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        
        # 캘리브레이션할 로봇 관절 각도 리스트
        self.pose_list = [
            [-83.23, -13.27, -15.02, -58.18, -0.08, -35.94],
            [-59.76, -12.56, -15.73, -64.42, -12.3, -22.41],
            [-51.76, -10.45, 5.97, -83.67, -13.09, 5.27],
            [-53.78, -20.91, -15.02, -64.16, -14.23, -9.31],
            [-51.94, -47.9, 5.88, -68.9, -15.2, -7.55],
            [-93.42, 5.71, -15.64, -64.07, 4.21, -15.73],
            [-83.4, 43.06, -37.08, -68.81, -6.5, -99.4],
            [-88.85, 0.87, -29.88, -52.2, 1.58, -38.84],
            [-88.15, -12.91, -15.02, -55.63, 0.26, -45.7],
            [-129.55, -32.51, 5.88, -61.87, 21.0, -86.57],
            [-78.92, 18.8, -14.23, -76.11, -3.69, -35.41],
            [-132.71, -63.54, 10.81, -52.55, 32.77, -90.96],
        ]

    def _init_robot(self):
        """로봇 초기화"""
        robot = MyCobot280(ROBOT_PORT, ROBOT_BAUDRATE)
        robot.thread_lock = True
        print("로봇이 연결되었습니다.")
        return robot

    def _init_camera(self):
        """카메라 초기화"""
        cap = cv2.VideoCapture(CAMERA_ID)
        if not cap.isOpened():
            raise IOError("카메라를 열 수 없습니다.")
        print("카메라가 연결되었습니다.")
        return cap

    def _init_network(self):
        """네트워크 설정 및 클라이언트 연결"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((SERVER_IP, SERVER_PORT))
        server_socket.listen(5)
        print(f"[TCP 서버] {SERVER_IP}:{SERVER_PORT} 대기 중...")
        
        client_socket, client_address = server_socket.accept()
        print(f"[TCP 서버] 클라이언트 연결됨: {client_address}")
        
        return server_socket, client_socket

    def flush_camera_buffer(self):
        """카메라 버퍼 비우기"""
        for _ in range(CAMERA_BUFFER_FRAMES):
            self.cap.read()
        print("[카메라] 버퍼 플러시 완료")

    def wait_for_robot(self, target_pose):
        """로봇이 목표 위치에 도달할 때까지 대기"""
        start_time = time.time()
        while time.time() - start_time < ROBOT_TIMEOUT:
            current_pose = self.robot.get_angles()
            if current_pose is None:
                time.sleep(0.5)
                continue
                
            diffs = [abs(c - t) for c, t in zip(current_pose, target_pose)]
            if max(diffs) < ROBOT_POSITION_THRESHOLD:
                return True
            time.sleep(0.5)
            
        print("[!] 로봇 이동 시간 초과!")
        return False

    def capture_and_process_frame(self):
        """프레임 캡처 및 체크보드 검출"""
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("프레임 캡처 실패")
            
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        found, corners = cv2.findChessboardCorners(rgb, CHECKERBOARD, None)
        
        if found:
            self.obj_points.append(self.obj_p)
            self.img_points.append(corners)
            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, found)
            
        return frame, found

    def send_frame(self, frame):
        """클라이언트에 프레임 전송"""
        data = pickle.dumps(frame)
        size = len(data)
        try:
            self.client_socket.sendall(struct.pack('>L', size))
            self.client_socket.sendall(data)
            return True
        except Exception as e:
            print(f"[!] 전송 중 오류 발생: {e}")
            return False

    def save_frame(self, frame, idx):
        """프레임 저장"""
        image_filename = os.path.join(SAVE_DIR, f"frame_{idx+1}.png")
        cv2.imwrite(image_filename, frame)
        return image_filename

    def perform_calibration(self, frame):
        """카메라 캘리브레이션 수행"""
        if not (self.obj_points and self.img_points):
            print("[!] 캘리브레이션을 위한 충분한 데이터가 없습니다.")
            return False
            
        print("[TCP 서버] 카메라 캘리브레이션 중...")
        frame_width = frame.shape[1]
        frame_height = frame.shape[0]
        
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, 
            self.img_points, 
            (frame_width, frame_height), 
            None, 
            None
        )
        
        if ret:
            print("[TCP 서버] 캘리브레이션 성공!")
            print("카메라 행렬:\n", camera_matrix)
            print("왜곡 계수:\n", dist_coeffs)
            return camera_matrix, dist_coeffs
        else:
            print("[TCP 서버] 캘리브레이션 실패!")
            return None

    def cleanup(self):
        """자원 해제"""
        self.cap.release()
        self.client_socket.close()
        self.server_socket.close()
        cv2.destroyAllWindows()
        print("[TCP 서버] 서버 종료")

    def run(self):
        """캘리브레이션 프로세스 실행"""
        try:
            last_frame = None
            
            for idx, pose in enumerate(self.pose_list):
                print(f"[{idx+1}/{len(self.pose_list)}] 위치로 이동 중: {pose}")
                
                # 로봇 이동
                self.robot.send_angles(pose, ROBOT_MOVE_SPEED)
                time.sleep(2)  # 로봇 이동 후 대기
                
                # 로봇 도착 확인
                if not self.wait_for_robot(pose):
                    print("[!] 로봇 이동 실패로 다음 위치로 스킵합니다.")
                    continue
                
                # 카메라 버퍼 비우기
                self.flush_camera_buffer()
                
                # 프레임 캡처 및 처리
                frame, found = self.capture_and_process_frame()
                last_frame = frame
                
                # 체크보드 검출 결과 출력
                if found:
                    print(f"[TCP 서버] 위치 {pose}에서 체크보드 코너 찾음.")
                else:
                    print(f"[TCP 서버] 위치 {pose}에서 체크보드 코너를 찾을 수 없음.")
                
                # 프레임 전송
                if not self.send_frame(frame):
                    break
                print(f"[TCP 서버] 위치 {pose}에서 프레임 전송 완료")
                
                # 프레임 저장
                image_filename = self.save_frame(frame, idx)
                print(f"[TCP 서버] 위치 {pose}에서 이미지 저장 완료: {image_filename}")
            
            # 캘리브레이션 수행
            if last_frame is not None:
                self.perform_calibration(last_frame)
                
        finally:
            # 자원 해제
            self.cleanup()


def main():
    """메인 함수"""
    try:
        calibrator = RobotCameraCalibration()
        calibrator.run()
    except Exception as e:
        print(f"[!] 오류 발생: {e}")


if __name__ == "__main__":
    main()