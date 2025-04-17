import cv2
import socket
import time

UDP_IP = "192.168.5.8"  # 👈 수신자 PC의 IP 주소
UDP_PORT = 12345        # 1024~65535 사이 PORT 개인 설정(0~1023은 시스템 예약 포트)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cap = cv2.VideoCapture('/dev/jetcocam0')  # Jetson 카메라 경로
print("📤 Start sending frames over UDP...")

if not cap.isOpened():
    print("❌ Failed to open camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # 크기 줄이기 (UDP 최대 패킷 제한 고려)
    frame = cv2.resize(frame, (640, 360))
    _, buffer = cv2.imencode('.jpg', frame)

    # UDP는 최대 65507 바이트 제한이 있어 나눠서 보내야 함
    data = buffer.tobytes()
    
    for i in range(0, len(data), 60000):
        chunk = data[i:i+60000]
        sock.sendto(chunk, (UDP_IP, UDP_PORT))
    time.sleep(1/30)
