import cv2
import numpy as np
import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("📥 Listening for UDP packets...")

buffer = b''
while True:
    data, _ = sock.recvfrom(65507) # 65507=64KB. UDP 최대 데이터 수신 크기
    buffer += data

    # 이미지 끝을 추정 (간단히 10KB 이상일 경우 처리)
    if len(buffer) > 10000:
        try:
            jpg = np.frombuffer(buffer, dtype=np.uint8)
            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)

            if frame is not None:
                cv2.imshow("UDP Video Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except Exception as e:
            print("⚠️ Error decoding frame:", e)
        buffer = b''

sock.close()
cv2.destroyAllWindows()
