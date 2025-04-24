import cv2
import socket
import struct
import pickle
import time
import logging
import numpy as np
from dt_apriltags import Detector
import jetcobot_utils.logger_config as logger_config


class ApriltagIdentify:
    def __init__(self):
        logger_config.setup_logger()
        self.image = None
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
        self.image = cv2.resize(image, (640, 480))
        msg = {}
        try:
            tags = self.at_detector.detect(cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY), estimate_tag_pose=False)
            tags = sorted(tags, key=lambda tag: tag.tag_id)

            for tag in tags:
                point_x, point_y = tag.center
                # 로봇 좌표계 변환 (임의 매핑: 사용 환경에 따라 수정 가능)
                a = round((point_x - 320) / 4000, 5)
                b = round(((480 - point_y) / 3000) * 0.8 + 0.15, 5)
                msg[tag.tag_id] = (a, b)

            # 태그 시각화
            self.image = self.draw_tags(self.image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))

        except Exception as e:
            logging.info(f'[Apriltag] getApriltagPosMsg error: {e}')

        return self.image, msg
    

    # ⭐ 핵심 코드
    def draw_tags(self, image, tags, corners_color=(255, 0, 0), center_color=(0, 255, 0)):
        for tag in tags:
            if tag.corners is not None and len(tag.corners) == 4:
                pts = tag.corners.astype(int).reshape((-1, 1, 2))
                cv2.polylines(image, [pts], isClosed=True, color=corners_color, thickness=5)
    
                center_x, center_y = int(tag.center[0]), int(tag.center[1])
                cv2.circle(image, (center_x, center_y), 5, center_color, -1)
    
                # 좌표 텍스트로 표시 (화면 좌표)
                coord_text = f"({center_x}, {center_y})"
                cv2.putText(image, coord_text, (center_x + 10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, center_color, 2)
    
        return image


def send_video_stream(host, port):
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 서버 재시작 시 포트 재사용 가능하게 설정
    server_socket.bind((host, port))
    server_socket.listen(5)
    print("📤 Waiting for connection...")
    client_socket, addr = server_socket.accept()
    print(f"✅ Connection from {addr} has been established!")

    tag_identify = ApriltagIdentify()

    try:
        while True:
            ret, frame = capture.read()
            if not ret:
                continue

            frame, data = tag_identify.getApriltagPosMsg(frame)

            # FPS 계산
            fps = int(capture.get(cv2.CAP_PROP_FPS))

            # FPS 표시
            cv2.putText(frame, f'FPS: {fps}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            # 프레임 직렬화
            data_serialized = pickle.dumps(frame)
            size = len(data_serialized)
            client_socket.sendall(struct.pack(">L", size))
            client_socket.sendall(data_serialized)

    except KeyboardInterrupt:
        pass
    finally:
        capture.release()
        client_socket.close()
        server_socket.close()


if __name__ == "__main__":
    send_video_stream('0.0.0.0', 8888)
