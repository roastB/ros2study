import streamlit as st
import asyncio
import websockets
import json

st.title("🥪 샌드위치 주문 입력")

# st.multiselect를 사용하여 다양하게 받을 수도 있음.
table = st.selectbox("테이블 번호", [1, 2, 3])
sandwich = st.selectbox("샌드위치 종류", ["불고기 샌드위치", "쉬림프 샌드위치", "베이컨 샌드위치"])
sauce = st.selectbox("소스 선택", ["이탈리안", "칠리"])
vegetable = st.selectbox("야채 선택", ["양상추", "로메인", "바질"])
cheese = st.selectbox("치즈 선택", ["슬라이스 치즈", "슈레드 치즈", "모짜렐라 치즈"])
price = st.number_input("최종 가격(수정필요!)", min_value=0)

if st.button("결제 완료"):
    data = {
        "table": table,
        "sandwich": sandwich,
        "sauce": sauce,
        "vegetable": vegetable,
        "cheese": cheese,
        "price": price
    }
    st.json(data)

    async def send_data():
        uri = "ws://localhost:6500"
        try:
            async with websockets.connect(uri) as websocket:
                await websocket.send(json.dumps(data))
                st.success("[✔] 서버로 전송 완료!")
        except Exception as e:
            st.error(f"[✕] 전송 실패: {e}")

    asyncio.run(send_data())
