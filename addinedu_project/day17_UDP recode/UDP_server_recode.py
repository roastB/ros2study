import cv2
import socket
import time

# 서버 설정
UDP_IP = "192.168.5.8"  # 클라이언트의 IP 주소
UDP_PORT = 12345        # 포트 번호

# 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 카메라 열기
cap = cv2.VideoCapture('/dev/video0')  # Jetson 카메라 경로

if not cap.isOpened():
    print("❌ Failed to open camera.")
    exit()

print("📤 Start sending frames over UDP...")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    frame = cv2.resize(frame, (640, 360))   # 크기 줄이기 (UDP 최대 패킷 제한 고려)
    _, buffer = cv2.imencode('.jpg', frame)
    data = buffer.tobytes()  # JPEG 포맷으로 인코딩된 데이터를 바이트 배열로 변환

    # 데이터를 여러 조각으로 나누어 보내기 (UDP 최대 크기 제한 65507 바이트)
    for i in range(0, len(data), 60000):
        chunk = data[i:i+60000]
        sock.sendto(chunk, (UDP_IP, UDP_PORT))
        print(f"Sent {len(chunk)} bytes")

    # 30FPS로 전송
    time.sleep(1/30)

cap.release()
sock.close()

