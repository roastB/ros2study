import socket
import threading

def handle_client(conn, addr):
    print(f"[클라이언트 연결됨] {addr}")
    
    # 연결 직후 환영 메시지 전송
    start_msg = "[SerboWay TCP/IP Start]"
    conn.sendall(start_msg.encode())
    
    while True:
        try:
            data = conn.recv(1024).decode()
            if not data:
                break
            print(f"[수신된 메시지] {addr}로부터: {data}")

            # 원하는 경우 여기서 응답을 다시 보낼 수도 있어
            # response = f"ACK: {data}"
            # conn.sendall(response.encode())
        except ConnectionResetError:
            print(f"[연결 끊김] {addr}")
            break
    conn.close()

def start_server():
    host = "0.0.0.0"
    port = 9999
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen()
    print(f"[서버 시작] {host}:{port} 에서 대기 중...")

    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        thread.start()

if __name__ == "__main__":
    start_server()
