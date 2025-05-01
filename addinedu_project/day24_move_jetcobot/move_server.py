import cv2
import socket
import struct
import pickle
import time
import logging
import numpy as np
from dt_apriltags import Detector
import jetcobot_utils.logger_config as logger_config
import os
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True
print("로봇이 연결되었습니다.")

speed = 50
test1 = [-81.91, -2.54, -118.21, 32.78, -1.75, -36.03]

# ===== 상수 정의 =====
# 움직임 및 거리 임계값 설정
TIME_THRESHOLD = 2  # 명령어 발행 사이의 최소 시간(초)
Z_VALUE_THRESHOLD = 0.0005  # 위치 변화 감지를 위한 Z값 변화 임계값(m)

# 로봇 제어 거리 임계값
VERY_CLOSE_THRESHOLD = 0.013  # 매우 가까운 거리 임계값(m)
CLOSE_THRESHOLD = 0.019  # 적당한 거리 임계값(m)

# 카메라 설정
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
TAG_SIZE = 0.02  # AprilTag 크기 (20mm)

# 카메라 캘리브레이션 매트릭스 (실제 카메라 캘리브레이션 결과)
CAMERA_MATRIX = np.array([
    [965.9166294, 0, 286.74986444],
    [0, 963.60630608, 165.32826283],
    [0, 0, 1]
])

DIST_COEFFS = np.array([
    [-3.81770715e-01, -4.12378540e-01, -4.33447987e-04, -1.65533463e-04, 1.79885791e+00]
])


# ===== 전역 변수 =====
# 최근 명령 시간 기억하는 딕셔너리와 태그 z값 기록
last_command_time = {}
tag_z_values = {}


def move_robot(tag_id, distance_z):
    """
    태그 ID와 거리 정보에 따라 JetCobot을 제어하는 함수
    """
    now = time.time()

    print(f"move_robot called: tag_id={tag_id}, distance_z={distance_z}")  # 디버깅 로그 추가

    if distance_z >= 0.17:
        print(f"Tag {tag_id} z값: {distance_z} - test1으로 이동")
        mc.send_angles(test1, speed)
        mc.set_gripper_value(100, speed)


class ApriltagIdentify:
    """
    AprilTag 감지 및 처리 클래스
    """
    def __init__(self):
        """
        ApriltagIdentify 클래스 초기화
        """
        # 로깅 설정
        logger_config.setup_logger()
        self.image = None

        # 카메라 파라미터 설정
        fx = CAMERA_MATRIX[0][0]
        fy = CAMERA_MATRIX[1][1]
        cx = CAMERA_MATRIX[0][2]
        cy = CAMERA_MATRIX[1][2]
        self.camera_params = (fx, fy, cx, cy)

        # AprilTag 감지기 초기화
        self.at_detector = Detector(
            searchpath=['apriltags'],
            families='tag36h11',
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

    def getApriltagPosMsg(self, image):
        """
        이미지에서 AprilTag를 감지하고 위치 정보를 반환
        
        Args:
            image (numpy.ndarray): 처리할 입력 이미지
            
        Returns:
            tuple: (처리된 이미지, 태그 위치 메시지 딕셔너리)
        """
        self.image = cv2.resize(image, (CAMERA_WIDTH, CAMERA_HEIGHT))
        msg = {}

        try:
            # 그레이스케일 변환 및 태그 감지
            gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            tags = self.at_detector.detect(
                gray_image,
                estimate_tag_pose=True,
                camera_params=self.camera_params,
                tag_size=TAG_SIZE
            )
            
            # 태그 ID 순으로 정렬
            tags = sorted(tags, key=lambda tag: tag.tag_id)

            for tag in tags:
                # 태그 중심 좌표 계산
                point_x, point_y = tag.center
                # 화면 중앙 기준 상대 좌표 계산
                normalized_x = round((point_x - CAMERA_WIDTH/2) / 4000, 5)
                normalized_y = round(((CAMERA_HEIGHT - point_y) / 3000) * 0.8 + 0.15, 5)
                msg[tag.tag_id] = (normalized_x, normalized_y)

                # 태그 거리 계산 및 로봇 제어
                if tag.pose_t is not None:
                    distance_z = tag.pose_t[2][0]
                    move_robot(tag.tag_id, distance_z)

            # 태그 정보를 이미지에 표시
            self.image = self.draw_tags(self.image, tags)

        except Exception as e:
            logging.error(f'[Apriltag] getApriltagPosMsg 에러: {e}')

        return self.image, msg

    def draw_tags(self, image, tags, corners_color=(255, 0, 0), center_color=(0, 255, 0)):
        """
        감지된 AprilTag 정보를 이미지에 시각화
        
        Args:
            image (numpy.ndarray): 표시할 원본 이미지
            tags (list): 감지된 AprilTag 목록
            corners_color (tuple): 태그 모서리 색상 (B,G,R)
            center_color (tuple): 태그 중심점 색상 (B,G,R)
            
        Returns:
            numpy.ndarray: 태그 정보가 표시된 이미지
        """
        for tag in tags:
            # 태그 모서리 그리기
            if tag.corners is not None and len(tag.corners) == 4:
                pts = tag.corners.astype(int).reshape((-1, 1, 2))
                cv2.polylines(image, [pts], isClosed=True, color=corners_color, thickness=5)

                # 태그 중심점 그리기
                center_x, center_y = int(tag.center[0]), int(tag.center[1])
                cv2.circle(image, (center_x, center_y), 5, center_color, -1)

                # 거리 정보 표시 (센티미터 단위로 변환)
                if tag.pose_t is not None:
                    distance_z = tag.pose_t[2][0]
                    # 미터를 센티미터로 변환 (100배)
                    distance_cm = distance_z * 100
                    cv2.putText(
                        image, 
                        f"Z: {distance_cm:.1f}cm", 
                        (center_x + 10, center_y + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.6, 
                        (0, 0, 255), 
                        2
                    )

                # 좌표 정보 표시
                coord_text = f"({center_x}, {center_y})"
                cv2.putText(
                    image, 
                    coord_text, 
                    (center_x + 10, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.6, 
                    center_color, 
                    2
                )

        return image


def send_video_stream(host='0.0.0.0', port=8888):
    """
    카메라 영상을 캡처하고 네트워크로 전송
    
    Args:
        host (str): 서버 호스트 주소
        port (int): 서버 포트 번호
    """
    # 카메라 초기화
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    # 소켓 서버 설정
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(5)
    
    print(f"[INFO] {host}:{port}에서 연결 대기 중...")
    client_socket, addr = server_socket.accept()
    print(f"[✔] {addr}에서 연결이 설정되었습니다!")

    # AprilTag 감지 객체 생성
    tag_identify = ApriltagIdentify()

    try:
        while True:
            # 프레임 캡처
            ret, frame = capture.read()
            if not ret:
                logging.warning("카메라에서 프레임을 읽지 못했습니다.")
                time.sleep(0.1)
                continue

            # 카메라 왜곡 보정
            frame = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS)

            # AprilTag 처리
            frame, tag_data = tag_identify.getApriltagPosMsg(frame)

            # FPS 표시
            fps = int(capture.get(cv2.CAP_PROP_FPS))
            cv2.putText(
                frame, 
                f'FPS: {fps}', 
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.9, 
                (0, 0, 255), 
                2
            )

            # 프레임 직렬화 및 전송
            try:
                data_serialized = pickle.dumps(frame)
                size = len(data_serialized)
                client_socket.sendall(struct.pack(">L", size))
                client_socket.sendall(data_serialized)
            except (ConnectionResetError, BrokenPipeError) as e:
                logging.error(f"연결 오류: {e}")
                break

    except KeyboardInterrupt:
        logging.info("사용자에 의해 프로그램이 중단되었습니다.")
    except Exception as e:
        logging.error(f"예상치 못한 오류 발생: {e}")
    finally:
        # 리소스 정리
        logging.info("리소스를 정리합니다...")
        capture.release()
        client_socket.close()
        server_socket.close()
        logging.info("프로그램 종료")


if __name__ == "__main__":
    send_video_stream()
