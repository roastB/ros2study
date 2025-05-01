import socket
import cv2
import pickle
import struct

def receive_video_stream(host, port):
    # 서버와 연결
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    print("[✔] Connected to server.")

    data = b""
    payload_size = struct.calcsize(">L")

    while True:
        while len(data) < payload_size:
            data += client_socket.recv(4096)
        packed_size = data[:payload_size]
        data = data[payload_size:]
        frame_size = struct.unpack(">L", packed_size)[0]

        while len(data) < frame_size:
            data += client_socket.recv(4096)
        frame_data = data[:frame_size]
        data = data[frame_size:]

        # 직렬화된 프레임을 복원하고 영상 출력
        frame = pickle.loads(frame_data)
        cv2.imshow("April Tag Detection (TCP Received Video)", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    client_socket.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    receive_video_stream('192.168.0.167', 8888)  # 라즈베리파이의 IP 주소로 변경