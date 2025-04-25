# pyqt_pc_gui_client.py
import sys
import socket
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel

class TCPClientGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt TCP Client")
        self.setGeometry(200, 200, 300, 200)

        # UI 구성
        self.label = QLabel("서버로부터 받은 메시지:", self)
        self.btn_send = QPushButton("서버에 'Test_Data' 전송", self)
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.btn_send)
        self.setLayout(layout)

        # TCP 연결
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(("192.168.0.178", 9999)) 

        # 수신 스레드 시작
        self.recv_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.recv_thread.start()

        # 버튼 클릭 시 실행될 함수 연결
        self.btn_send.clicked.connect(self.send_command)
    
    def send_command(self):
        try:
            message = "SerboWay_Test_Data"
            self.client.sendall(message.encode())
            self.label.setText(f"보낸 명령: {message}")

        except Exception as e:
            self.label.setText(f"전송 실패: {e}")
    
    def receive_data(self):
        while True:
            try:
                data = self.client.recv(1024).decode()
                if data:
                    self.label.setText(f"서버 응답: {data}")

            except Exception as e:
                print(f"[RECV ERROR] {e}")
                break
                
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = TCPClientGUI()
    gui.show()
    sys.exit(app.exec_())