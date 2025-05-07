# udp_cam_sender.py (JetCobot 측)
import socket
import cv2
import time

UDP_IP = "192.168.5.8"
UDP_PORT = 9999
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)

frame_id = 0
print("[Sender] 전송 시작...")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # JPEG 인코딩
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    img_data = buffer.tobytes()

    if len(img_data) > 65000 - 4:
        print(f"[경고] 프레임 크기 {len(img_data)} byte: UDP 최대 초과!")
        continue

    # 4바이트 프레임 순번 헤더 추가
    frame_id += 1
    header = frame_id.to_bytes(4, byteorder='big')  # 상위 4바이트에 순번
    sock.sendto(header + img_data, (UDP_IP, UDP_PORT))

    print(f"[전송] Frame {frame_id} | {len(img_data)} byte")
    time.sleep(0.033)
