# client_calibration_capture.py
import socket
import cv2
import struct
import pickle
import numpy as np

HOST = '192.168.0.167' 
PORT = 8485

chessboard_size = (9, 6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

obj_points = []
img_points = []

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
data = b""

calibration_frame_count = 20
frame_count = 0

while True:
    # 수신
    while len(data) < 4:
        data += sock.recv(4 - len(data))
    msg_size = struct.unpack(">L", data[:4])[0]
    data = data[4:]

    while len(data) < msg_size:
        data += sock.recv(msg_size - len(data))

    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame = pickle.loads(frame_data)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        obj_points.append(objp)
        img_points.append(corners2)
        frame_count += 1
        print(f"[✓] Frame {frame_count} captured")

        cv2.drawChessboardCorners(frame, chessboard_size, corners2, ret)

    cv2.imshow("Live Calibration Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q') or frame_count >= calibration_frame_count:
        break

sock.close()
cv2.destroyAllWindows()

# 캘리브레이션 실행
if frame_count > 0:
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None)
    
    print("===[ Calibration Result ]===")
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion Coefficients:\n", dist_coeffs)
else:
    print("No valid frames collected.")
