import asyncio
import websockets
import cv2
import base64

connected_clients = set()
current_frame = None
frame_lock = asyncio.Lock()

# ✅ 싱글 프로듀서 : 카메라 프레임을 계속 생성해서 공유 변수에 저장
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
            continue

        _, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')

        async with frame_lock:
            current_frame = jpg_as_text

        await asyncio.sleep(1/30)  # 약 30fps

# ✅ 멀티 컨슈머 : 클라이언트에게 현재 프레임을 계속 전송
async def send_frames(websocket):
    print("🟩[INFO] 클라이언트 연결")
    connected_clients.add(websocket)

    try:
        while True:
            async with frame_lock:
                if current_frame:
                    await websocket.send(current_frame)
            await asyncio.sleep(1/30)
    except websockets.exceptions.ConnectionClosed:
        print("🟥[INFO] 클라이언트 연결 종료")
    finally:
        connected_clients.remove(websocket)

# ✅ 메인 서버 실행부
async def main():
    print("🛜[INFO] WebSocket 서버 시작됨 (port 8765)")
    server = await websockets.serve(send_frames, "0.0.0.0", 8765)

    # 프레임 생성은 별도의 Task로 실행
    await frame_producer()

    # WebSocket 서버 무한 대기
    await server.wait_closed()

asyncio.run(main())

