# [통신 레이싱 이슈란?]
# 둘 이상의 클라이언트나 쓰레드/프로세스가 동일한 리소스에 동시에 접근하려 할 때,
# 그 결과가 실행 순서에 따라 달라지는 현상을 말해.
# ex) 두 명 이상이 동시에 카메라를 열면 충돌하거나, 영상이 깨지는 현상

# [현재 문제점]
# 이 코드는 클라이언트가 접속할 때마다 send_frames()가 호출되며
# 내부에서 독립적으로 cv2.VideoCapture('/dev/jetcocam0')를 열기 때문에
# 여러 명이 접속하면 카메라 장치를 여러 번 열게 되고, 충돌이 발생할 수 있음 (레이싱 이슈)

import asyncio
import websockets
import cv2
import base64

async def send_frames(websocket):
    print(f"[INFO] 클라이언트 연결됨")

    # ❌ 클라이언트가 연결될 때마다 카메라를 새로 엶
    # -> 여러 명이 동시에 연결되면 카메라 자원 충돌 (레이싱 이슈 발생 가능)
    cap = cv2.VideoCapture('/dev/jetcocam0')

    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # 프레임을 JPEG로 인코딩 후 Base64로 변환해서 전송
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            await websocket.send(jpg_as_text)
            await asyncio.sleep(1/30)  # 약 30fps
    except websockets.exceptions.ConnectionClosed:
        print("[INFO] 클라이언트 연결 종료됨.")
    finally:
        cap.release()  # 이 클라이언트가 연결 종료되면 카메라도 닫힘

# 🔧 여기부터 수정됨
async def main():
    print("[INFO] WebSocket 서버 시작됨 (port 8765)")

    # ✅ 하지만 여전히 send_frames()가 클라이언트 수만큼 실행되므로
    #    각 클라이언트가 카메라를 따로 열게 됨 -> 여전히 레이싱 이슈 있음

    async with websockets.serve(send_frames, "0.0.0.0", 8765):
        await asyncio.Future()  # 무한 대기

asyncio.run(main())

# [해결방안 요약 - 추후 적용해야 할 구조]
# 1. 카메라를 하나만 열고, 프레임을 전역적으로 유지 (싱글 프로듀서)
# 2. 각 클라이언트는 그 전역 프레임을 가져가서 보기만 함 (멀티 컨슈머)
# 3. 필요시 asyncio.Lock을 활용해서 frame 접근 순서 조정

