import cv2
import socket
import numpy as np  # NumPy 모듈 임포트 추가

# 서버 IP 및 포트 설정
UDP_IP = '0.0.0.0'  # 모든 네트워크 인터페이스에서 수신
UDP_PORT = 12345     # 서버와 동일한 포트 번호

# 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))  # 포트 바인딩

print(f"Listening on {UDP_IP}:{UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(60000)  # 최대 60,000 바이트씩 수신
    print(f"Received packet from {addr}")

    # 데이터가 있으면 프레임으로 변환하여 화면에 표시
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    if frame is not None:
        cv2.imshow("Received Frame", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
sock.close()
