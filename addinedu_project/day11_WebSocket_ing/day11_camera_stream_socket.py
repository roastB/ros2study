import asyncio
import websockets
import cv2
import base64
import numpy as np

async def video_stream(websocket, path):
    cap = cv2.VideoCapture(0)  # 카메라 열기
    if not cap.isOpened():
        print("❌ 카메라를 열 수 없습니다.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임을 읽을 수 없습니다.")
            break

        # OpenCV 이미지를 base64로 인코딩하여 웹소켓을 통해 전송
        _, jpeg = cv2.imencode('.jpg', frame)
        jpeg_bytes = jpeg.tobytes()
        jpeg_base64 = base64.b64encode(jpeg_bytes).decode('utf-8')

        await websocket.send(jpeg_base64)  # Base64로 인코딩된 이미지 전송

        await asyncio.sleep(0.1)  # 서버가 너무 빨리 응답하지 않도록 슬립을 줍니다.

    cap.release()  # 카메라 자원 해제

# 웹소켓 서버를 asyncio 이벤트 루프에서 실행
async def start_server():
    server = await websockets.serve(video_stream, "0.0.0.0", 8766)
    print("📡 WebSocket 서버 실행 중 (ws://0.0.0.0:8766)")
    await server.wait_closed()

# 이벤트 루프 시작
if __name__ == "__main__":
    asyncio.run(start_server())

