# jetson_camera_tcp_server.py
import socket
import cv2
import struct
import pickle

cam = cv2.VideoCapture(0)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 8485))
server_socket.listen(1)
print("Server ready. Waiting for client...")

conn, addr = server_socket.accept()
print(f"Connected to {addr}")

while True:
    ret, frame = cam.read()
    if not ret:
        break

    data = pickle.dumps(frame)
    size = len(data)
    conn.sendall(struct.pack(">L", size) + data)

cam.release()
conn.close()
server_socket.close()
