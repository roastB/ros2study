import asyncio
import websockets
import json

async def handle_connection(websocket):
    print("[✔] 클라이언트 연결됨")
    try:
        async for message in websocket:
            # 클라이언트로부터 받은 메시지를 JSON으로 파싱
            data = json.loads(message)
            print(f"[▼] 받은 데이터: {data}")
            
            # 데이터 처리 후 응답 (필요시 추가)
            # response = {"status": "success", "message": "데이터 처리 완료"}
            # await websocket.send(json.dumps(response))  # 클라이언트로 응답 보내기
            
    except websockets.exceptions.ConnectionClosed:
        print("[✕] 클라이언트 연결 종료됨")

async def main():
    server = await websockets.serve(handle_connection, "0.0.0.0", 6500)
    print("[✔] WebSocket 서버 실행 중: ws://localhost:6500")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())