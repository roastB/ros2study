from flask import Flask, request, jsonify
import mysql.connector
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

app = Flask(__name__)

def get_db_connection():
    try:
        conn = mysql.connector.connect(
            host="****",
            user="serboway",
            password="****",
            database="****"
        )
        return conn
    except mysql.connector.Error as err:
        logging.error(f"Database connection error: {err}")
        return None

@app.route('/order', methods=['POST'])
def receive_order():
    # Get JSON data from request
    try:
        data = request.get_json()
        if not data:
            return jsonify({"message": "데이터가 없습니다"}), 400
        
        logging.info(f"주문 수신: {data}")
        
        # Extract data with validation
        table_number = data.get("table_number")
        sandwich = data.get("sandwich")
        sauce = data.get("sauce")
        vegetable = data.get("vegetable")
        cheese = data.get("cheese")
        price = data.get("price")
        
        # Basic validation
        if None in [table_number, sandwich, sauce, vegetable, cheese, price]:
            missing_fields = [field for field, value in 
                             zip(["table_number", "sandwich", "sauce", "vegetable", "cheese", "price"],
                                 [table_number, sandwich, sauce, vegetable, cheese, price]) 
                             if value is None]
            return jsonify({
                "message": "필수 필드가 누락되었습니다", 
                "missing_fields": missing_fields
            }), 400

        # Store in database
        conn = get_db_connection()
        if not conn:
            return jsonify({"message": "데이터베이스 연결 실패"}), 500
        
        try:
            cursor = conn.cursor()
            
            # created_at은 자동으로 CURRENT_TIMESTAMP 설정됨
            query = """
                INSERT INTO orders (table_number, sandwich, sauce, vegetable, cheese, price)
                VALUES (%s, %s, %s, %s, %s, %s)
            """
            cursor.execute(query, (table_number, sandwich, sauce, vegetable, cheese, price))
            conn.commit()
            
            order_id = cursor.lastrowid
            logging.info(f"주문 #{order_id} 저장 완료")
            
            return jsonify({
                "message": "주문이 성공적으로 처리되었습니다",
                "order_id": order_id
            }), 200
            
        except mysql.connector.Error as err:
            logging.error(f"MySQL 오류: {err}")
            return jsonify({"message": "DB 저장 실패", "error": str(err)}), 500
        
        finally:
            if 'cursor' in locals():
                cursor.close()
            if conn.is_connected():
                conn.close()
                
    except Exception as e:
        logging.error(f"서버 오류: {str(e)}")
        return jsonify({"message": "서버 오류", "error": str(e)}), 500

@app.route('/health', methods=['GET'])
def health_check():
    """Simple endpoint to verify the server is running"""
    return jsonify({"status": "ok", "message": "서버가 정상적으로 실행 중입니다"}), 200

if __name__ == '__main__':
    logging.info("샌드위치 주문 서버 시작 중...")
    app.run(host="0.0.0.0", port=5003)
    logging.info("서버 종료")
