import streamlit as st
import json
import requests
import time
from datetime import datetime
import re

# Page configuration
st.set_page_config(
    page_title="서보웨이 주문 시스템",
    page_icon="🥪",
    layout="wide"
)

# Custom CSS
st.markdown("""
<style>
    .main-header {
        font-size: 2.5rem;
        color: #0f766e;
        text-align: center;
    }
    .subheader {
        font-size: 1.5rem;
        color: #0f766e;
        margin-top: 1rem;
    }
    .success-message {
        background-color: #d1fae5;
        padding: 1rem;
        border-radius: 0.5rem;
        text-align: center;
        margin: 1rem 0;
    }
    .error-message {
        background-color: #fee2e2;
        padding: 1rem;
        border-radius: 0.5rem;
        text-align: center;
        margin: 1rem 0;
    }
    .stButton button {
        background-color: #0f766e;
        color: white;
        padding: 0.5rem 1rem;
    }
</style>
""", unsafe_allow_html=True)

# App title
st.markdown("<h1 class='main-header'>🥪 서보웨이 샌드위치 주문 시스템</h1>", unsafe_allow_html=True)

# Initialize session state
if 'order_submitted' not in st.session_state:
    st.session_state.order_submitted = False
if 'order_id' not in st.session_state:
    st.session_state.order_id = None
if 'server_url' not in st.session_state:
    st.session_state.server_url = "http://localhost:5003"

# Server configuration
with st.sidebar:
    st.markdown("<h2 class='subheader'>⚙️ 서버 설정</h2>", unsafe_allow_html=True)
    server_url = st.text_input("서버 URL", value=st.session_state.server_url)
    st.session_state.server_url = server_url
    
    # Server health check
    if st.button("서버 연결 확인"):
        try:
            response = requests.get(f"{server_url}/health", timeout=5)
            if response.status_code == 200:
                st.success("✅ 서버 연결 성공!")
            else:
                st.error(f"❌ 서버 응답 코드: {response.status_code}")
        except Exception as e:
            st.error(f"❌ 서버 연결 실패: {e}")
    
    st.divider()
    st.markdown("### 🏢 서보웨이 샌드위치")
    st.write("최고의 맛과 건강을 드립니다.")
    st.write("© 2025 서보웨이")

# Helper function to extract price from options with price tags
def extract_price(option_text):
    price_match = re.search(r'\+([0-9,]+)원', option_text)
    if price_match:
        price_str = price_match.group(1).replace(',', '')
        return int(price_str)
    return 0

# Main order form
if not st.session_state.order_submitted:
    # Create two columns for form layout
    col1, col2 = st.columns(2)
    
    with col1:
        st.markdown("<h2 class='subheader'>📋 기본 정보</h2>", unsafe_allow_html=True)
        table = st.selectbox("테이블 번호", options=[1, 2, 3])
        sandwich = st.selectbox(
            "샌드위치 종류", 
            options=[
                "불고기 샌드위치", 
                "쉬림프 샌드위치", 
                "베이컨 샌드위치"
            ]
        )
        
    with col2:
        st.markdown("<h2 class='subheader'>🥗 디테일 옵션</h2>", unsafe_allow_html=True)
        sauce = st.selectbox(
            "소스 선택", 
            options=[
                "이탈리안", 
                "칠리", 
                "스위트 어니언"
            ]
        )
        vegetable = st.selectbox(
            "야채 선택", 
            options=[
                "양상추", 
                "로메인 (+700원)", 
                "바질 (+800원)"
            ]
        )
        cheese = st.selectbox(
            "치즈 선택", 
            options=[
                "슬라이스 치즈", 
                "슈레드 치즈 (+1,000원)", 
                "모짜렐라 치즈 (+1,300원)"
            ]
        )
    
    # Pricing based on sandwich type
    price_map = {
        "불고기 샌드위치": 6500,
        "쉬림프 샌드위치": 6200,
        "베이컨 샌드위치": 6000
    }
    
    # Calculate base price for the sandwich
    base_price = price_map.get(sandwich, 7000)
    
    # Calculate additional costs for vegetable options
    vegetable_price = extract_price(vegetable)
    
    # Calculate additional costs for cheese options
    cheese_price = extract_price(cheese)
    
    # Calculate final price
    final_price = base_price + vegetable_price + cheese_price

    # Display order summary
    st.markdown("<h2 class='subheader'>💰 주문 요약</h2>", unsafe_allow_html=True)
    col3, col4 = st.columns(2)
    
    with col3:
        st.write(f"**기본 가격:** {base_price:,}원")
        if vegetable_price > 0:
            st.write(f"- 야채 추가 비용: +{vegetable_price:,}원")
        if cheese_price > 0:
            st.write(f"- 치즈 추가 비용: +{cheese_price:,}원")
    
    with col4:
        st.metric(label="최종 가격", value=f"{final_price:,}원")
        
        # Order button
        if st.button("✅ 결제 완료 및 주문하기", use_container_width=True):
            # Extract clean option names (without price tags)
            vegetable_name = vegetable.split(" (+")[0] if " (+" in vegetable else vegetable
            cheese_name = cheese.split(" (+")[0] if " (+" in cheese else cheese
            
            # Prepare data for sending
            order_data = {
                "table_number": table,
                "sandwich": sandwich,
                "sauce": sauce,
                "vegetable": vegetable_name,  # Clean name without price
                "cheese": cheese_name,        # Clean name without price
                "price": final_price
            }
            
            # Send to server
            try:
                with st.spinner("주문 처리 중..."):
                    response = requests.post(
                        f"{st.session_state.server_url}/order", 
                        json=order_data,
                        timeout=10
                    )
                
                if response.status_code == 200:
                    resp_data = response.json()
                    st.session_state.order_id = resp_data.get("order_id")
                    st.session_state.order_submitted = True
                    st.session_state.order_details = order_data
                    st.session_state.order_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    st.rerun()
                else:
                    st.markdown(
                        f"<div class='error-message'>❌ 주문 처리 실패: {response.status_code}<br>{response.text}</div>", 
                        unsafe_allow_html=True
                    )
            except Exception as e:
                st.markdown(
                    f"<div class='error-message'>❌ 서버 연결 오류: {str(e)}</div>", 
                    unsafe_allow_html=True
                )

# Order confirmation page
else:
    st.markdown("<h2 class='subheader'>📝 주문 확인</h2>", unsafe_allow_html=True)
    
    # Success message
    st.markdown(
        f"""
        <div class='success-message'>
            <h3>✅ 주문이 완료되었습니다!</h3>
            <p>주문 번호: #{st.session_state.order_id}</p>
            <p>주문 시간: {st.session_state.order_time}</p>
        </div>
        """, 
        unsafe_allow_html=True
    )
    
    # Order details
    col5, col6 = st.columns(2)
    
    with col5:
        st.markdown("<h3>주문 내역</h3>", unsafe_allow_html=True)
        order = st.session_state.order_details
        st.write(f"**테이블 번호:** {order['table_number']}")
        st.write(f"**샌드위치:** {order['sandwich']}")
        st.write(f"**소스:** {order['sauce']}")
        st.write(f"**야채:** {order['vegetable']}")
        st.write(f"**치즈:** {order['cheese']}")
    
    with col6:
        st.markdown("<h3>결제 정보</h3>", unsafe_allow_html=True)
        st.metric(label="결제 금액", value=f"{order['price']:,}원")
        st.write("**결제 방법:** 카드 결제")
        st.write("**영수증 번호:** SBW-" + datetime.now().strftime("%Y%m%d") + f"-{st.session_state.order_id}")
    
    # Estimated prep time
    st.info("예상 준비 시간: 약 10-15분")
    
    # New order button
    if st.button("🔄 새 주문 시작하기", use_container_width=True):
        st.session_state.order_submitted = False
        st.session_state.order_id = None
        st.rerun()