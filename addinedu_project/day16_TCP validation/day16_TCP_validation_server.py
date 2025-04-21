import asyncio
import websockets
import cv2
import base64
import time

connected_clients = set()
current_frame = None
frame_lock = asyncio.Lock()

# 카메라 프레임 생성
async def frame_producer():
    global current_frame
    cap = cv2.VideoCapture('/dev/jetcocam0')

    if not cap.isOpened():
        print("⚠️[ERROR] 카메라를 열 수 없습니다.")
        return

    print("🟦[INFO] 프레임 생성 시작")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️[WARNING] 프레임 읽기 실패")
            continue

        _, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')

        async with frame_lock:
            current_frame = jpg_as_text

        await asyncio.sleep(1 / 30)  # 30fps

# 클라이언트에게 프레임 전송
async def send_frames(websocket):
    print("🟩[INFO] 클라이언트 연결")
    connected_clients.add(websocket)

    try:
        while True:
            async with frame_lock:
                if current_frame:
                    start = time.time()
                    await websocket.send(current_frame)
                    end = time.time()
                    print(f"📡 프레임 전송 시간: {round((end - start)*1000, 2)}ms")
            await asyncio.sleep(1 / 30)

    except websockets.exceptions.ConnectionClosed:
        print("🟥[INFO] 클라이언트 연결 종료")

    finally:
        connected_clients.remove(websocket)

# 서버 실행
async def main():
    print("🛜[INFO] WebSocket 서버 시작됨 (port 8765)")
    server = await websockets.serve(send_frames, "0.0.0.0", 8765)
    await frame_producer()
    await server.wait_closed()

asyncio.run(main())
