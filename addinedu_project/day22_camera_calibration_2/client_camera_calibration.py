import cv2
import socket
import pickle
import struct

# 서버 IP와 포트 설정
server_ip = '192.168.0.167'  # 서버 IP
server_port = 8889  # 포트 번호

# 소켓 설정
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_ip, server_port))
print(f"[TCP 클라이언트] 서버 {server_ip}:{server_port}에 연결되었습니다.")

# 이미지 번호 표시
image_count = 0

# 데이터 수신 루프
while True:
    try:
        # 메시지 크기 수신
        msg_size = struct.unpack(">L", client_socket.recv(struct.calcsize(">L")))[0]
        
        # 데이터 수신
        data = b""
        while len(data) < msg_size:
            data += client_socket.recv(4096)
        
        # 받은 데이터를 디코딩하여 이미지로 변환
        frame = pickle.loads(data)
        
        # 받은 이미지를 화면에 출력
        image_count += 1
        cv2.imshow(f"Frame {image_count}", frame)
        
        # 'q' 키를 눌러 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"[TCP 클라이언트] 오류 발생: {e}")
        break

# 자원 해제
client_socket.close()
cv2.destroyAllWindows()
