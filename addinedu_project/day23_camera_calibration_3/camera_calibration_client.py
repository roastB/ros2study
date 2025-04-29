#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
로봇 카메라 캘리브레이션 클라이언트
서버에서 전송한 카메라 이미지를 수신하고 화면에 표시하는 프로그램
"""

import cv2
import socket
import pickle
import struct
import time

# 상수 정의
SERVER_IP = '192.168.0.167'
SERVER_PORT = 8889
BUFFER_SIZE = 4096
MAX_RETRY_COUNT = 3
RETRY_WAIT_TIME = 2


class CalibrationClient:
    """카메라 캘리브레이션 클라이언트 클래스"""
    
    def __init__(self, server_ip, server_port):
        """클래스 초기화"""
        self.server_ip = server_ip
        self.server_port = server_port
        self.client_socket = None
        self.image_count = 0
        self.is_connected = False
    
    def connect(self):
        """서버에 연결"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.server_ip, self.server_port))
            self.is_connected = True
            print(f"[TCP 클라이언트] 서버 {self.server_ip}:{self.server_port}에 연결되었습니다.")
            return True
        except Exception as e:
            print(f"[TCP 클라이언트] 연결 실패: {e}")
            return False
    
    def receive_data_safely(self, size):
        """지정된 크기의 데이터를 안전하게 수신"""
        data = b""
        remaining = size
        
        while remaining > 0:
            chunk = self.client_socket.recv(min(remaining, BUFFER_SIZE))
            if not chunk:  # 연결이 끊어진 경우
                raise ConnectionError("서버와의 연결이 끊어졌습니다.")
            data += chunk
            remaining -= len(chunk)
        
        return data
    
    def receive_frame(self):
        """서버로부터 프레임 수신"""
        try:
            # 메시지 크기 수신 (4바이트)
            size_data = self.receive_data_safely(struct.calcsize(">L"))
            if not size_data:
                return None
                
            msg_size = struct.unpack(">L", size_data)[0]
            
            # 이미지 데이터 수신
            data = self.receive_data_safely(msg_size)
            
            # 받은 데이터를 디코딩하여 이미지로 변환
            frame = pickle.loads(data)
            self.image_count += 1
            
            print(f"[TCP 클라이언트] 프레임 {self.image_count} 수신 완료 (크기: {msg_size} 바이트)")
            return frame
            
        except ConnectionError as e:
            print(f"[TCP 클라이언트] 연결 오류: {e}")
            return None
        except struct.error as e:
            print(f"[TCP 클라이언트] 데이터 구조 오류: {e}")
            return None
        except pickle.UnpicklingError as e:
            print(f"[TCP 클라이언트] 데이터 역직렬화 오류: {e}")
            return None
        except Exception as e:
            print(f"[TCP 클라이언트] 예상치 못한 오류: {e}")
            return None
    
    def display_frame(self, frame):
        """프레임 화면에 표시"""
        if frame is not None:
            window_name = f"캘리브레이션 프레임 {self.image_count}"
            cv2.imshow(window_name, frame)
            
            # 창 위치 조정 (화면 중앙에 배치)
            cv2.moveWindow(window_name, 100, 100)
            
            return cv2.waitKey(1) & 0xFF
        return None
    
    def save_frame(self, frame, directory="./received_frames"):
        """프레임 저장"""
        import os
        os.makedirs(directory, exist_ok=True)
        
        filename = f"{directory}/frame_{self.image_count}.png"
        cv2.imwrite(filename, frame)
        print(f"[TCP 클라이언트] 프레임 저장: {filename}")
    
    def close(self):
        """자원 해제"""
        if self.client_socket:
            self.client_socket.close()
            print("[TCP 클라이언트] 소켓 연결 종료")
        
        cv2.destroyAllWindows()
        print("[TCP 클라이언트] 모든 창 종료")
    
    def run(self):
        """클라이언트 실행"""
        if not self.connect():
            return
        
        print("[TCP 클라이언트] 서버로부터 프레임 수신 대기 중...")
        
        try:
            while True:
                frame = self.receive_frame()
                
                if frame is None:
                    print("[TCP 클라이언트] 더 이상 프레임을 수신할 수 없습니다.")
                    break
                
                # 옵션: 프레임 저장
                # self.save_frame(frame)
                
                # 프레임 표시 및 키 입력 확인
                key = self.display_frame(frame)
                if key == ord('q'):
                    print("[TCP 클라이언트] 사용자에 의해 종료")
                    break
                
        except KeyboardInterrupt:
            print("[TCP 클라이언트] 키보드 인터럽트에 의해 종료")
        finally:
            self.close()


def main():
    """메인 함수"""
    retry_count = 0
    
    while retry_count < MAX_RETRY_COUNT:
        try:
            client = CalibrationClient(SERVER_IP, SERVER_PORT)
            client.run()
            break  # 정상적으로 실행 완료
            
        except Exception as e:
            retry_count += 1
            print(f"[TCP 클라이언트] 오류 발생: {e}")
            print(f"[TCP 클라이언트] 재시도 중... ({retry_count}/{MAX_RETRY_COUNT})")
            time.sleep(RETRY_WAIT_TIME)
    
    if retry_count >= MAX_RETRY_COUNT:
        print("[TCP 클라이언트] 최대 재시도 횟수 초과. 프로그램을 종료합니다.")


if __name__ == "__main__":
    main()