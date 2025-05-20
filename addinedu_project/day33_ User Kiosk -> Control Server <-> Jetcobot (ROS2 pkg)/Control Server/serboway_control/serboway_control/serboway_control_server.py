#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import os
from std_msgs.msg import String
from my_msgs.msg import OrderInformation, CompleteOrder
import mysql.connector
import logging
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import sys

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class SerbowayControlServer(Node):
    def __init__(self):
        super().__init__('serboway_control_server')
        
        # 주문 전송 퍼블리셔 생성
        self.order_publisher = self.create_publisher(
            OrderInformation, 
            'control_jetcobot',  # 주문 정보를 jetcobot에 pub 위한 토픽 이름
            10)
        
        self.signal_publisher = self.create_publisher(
            String,
            'making_signal',  # 완료 신호를 위한 토픽 이름
            10)
        
        # Jetcobot 완료 신호 구독자 추가
        self.completion_subscription = self.create_subscription(
            CompleteOrder,
            'complete_control',
            self.completion_callback,
            10
        )

        # JSON 파일 저장 디렉토리
        self.json_dir = os.path.expanduser('~/order_data')
        os.makedirs(self.json_dir, exist_ok=True)
        
        # Flask 서버를 별도 스레드에서 실행
        self.flask_thread = threading.Thread(target=self.run_flask_server)
        self.flask_thread.daemon = True  # 메인 스레드가 종료되면 이 스레드도 종료
        self.flask_thread.start()
        
        self.get_logger().info('서보웨이 컨트롤 서버 초기화 완료')
    
    def get_db_connection(self):
        try:
            conn = mysql.connector.connect(
                host="localhost",
                user="serboway",
                password="4011",
                database="serboway_db",
                use_pure=True,  # 네이티브 C 익스텐션 대신 순수 Python 구현 사용
                ssl_disabled=True  # SSL 비활성화
            )
            return conn
        except mysql.connector.Error as err:
            self.get_logger().error(f"Database connection error: {err}")
            return None
    
    def process_order(self, order_data):
        """주문 데이터를 처리하고 ROS 토픽으로 발행"""
        try:
            # JSON 파일로 저장
            order_id = order_data.get('order_id', 0)
            json_path = os.path.join(self.json_dir, f"order_{order_id}.json")
            with open(json_path, 'w') as f:
                json.dump(order_data, f, indent=4)

            self.get_logger().info(f'주문 데이터를 {json_path}에 저장했습니다')

            # OrderInformation 메시지 생성
            order_msg = OrderInformation()

            # id 설정 (정수형으로 변환)
            try:
                order_msg.id = int(order_id)
            except (ValueError, TypeError):
                order_msg.id = 0  # 기본값

            # ingredients 배열 설정 (4개 항목)
            ingredients = ["", "", "", ""]  # 기본값 설정

            # 0: 샌드위치
            if 'sandwich' in order_data and order_data['sandwich']:
                ingredients[0] = str(order_data['sandwich'])

            # 1: 소스
            if 'sauce' in order_data and order_data['sauce']:
                ingredients[1] = str(order_data['sauce'])

            # 2: 야채
            if 'vegetable' in order_data and order_data['vegetable']:
                ingredients[2] = str(order_data['vegetable'])

            # 3: 치즈
            if 'cheese' in order_data and order_data['cheese']:
                ingredients[3] = str(order_data['cheese'])

            order_msg.ingredients = ingredients

            # 주문 정보 발행
            self.order_publisher.publish(order_msg)
            self.get_logger().info(f'주문 정보를 발행했습니다: {order_id}')

            # 완료 신호 발행
            self.publish_completion_signal()

            return True

        except Exception as e:
            self.get_logger().error(f'주문 처리 중 오류 발생: {str(e)}')
            return False

    def publish_completion_signal(self):
        """Jetcobot를 트리거하기 위한 완료 신호 발행"""
        signal_msg = String()
        signal_msg.data = "Making completed!"
        self.signal_publisher.publish(signal_msg)
        self.get_logger().info('완료 신호를 발행했습니다')
    
    def completion_callback(self, msg):
           """Jetcobot에서 보내는 제작 완료 신호 처리"""
           self.get_logger().info(f"Jetcobot 신호 수신: 주문 ID {msg.id}, 상태: {msg.order_completed}")

           if msg.order_completed == "Making Completed!":
               # 주문 완료 처리
               self.update_order_status(msg.id, "completed")
           elif msg.order_completed == "Making Failed":
               # 주문 실패 처리
               self.update_order_status(msg.id, "failed")
    
    def update_order_status(self, order_id, status):
        """주문 상태 업데이트"""
        try:
            conn = self.get_db_connection()
            if not conn:
                self.get_logger().error("데이터베이스 연결 실패")
                return
            
            try:
                cursor = conn.cursor()
                
                # orders 테이블에 status 컬럼 존재 여부 확인
                # 없다면 먼저 ALTER TABLE로 추가해야 함
                try:
                    check_query = """
                        SELECT COUNT(*) 
                        FROM information_schema.COLUMNS 
                        WHERE TABLE_SCHEMA = %s
                        AND TABLE_NAME = 'orders' 
                        AND COLUMN_NAME = 'status'
                    """
                    cursor.execute(check_query, (conn.database,))
                    column_exists = cursor.fetchone()[0] > 0
                    
                    if not column_exists:
                        self.get_logger().info("orders 테이블에 status 컬럼 추가 중...")
                        alter_query = """
                            ALTER TABLE orders 
                            ADD COLUMN status VARCHAR(50) DEFAULT 'pending'
                        """
                        cursor.execute(alter_query)
                        conn.commit()
                        self.get_logger().info("status 컬럼이 추가되었습니다")
                except mysql.connector.Error as err:
                    self.get_logger().error(f"컬럼 확인/추가 중 오류: {err}")
                    return
                
                # 상태 업데이트
                query = """
                    UPDATE orders 
                    SET status = %s
                    WHERE id = %s
                """
                cursor.execute(query, (status, order_id))
                conn.commit()
                
                self.get_logger().info(f"주문 #{order_id} 상태가 '{status}'로 업데이트되었습니다")
                
            except mysql.connector.Error as err:
                self.get_logger().error(f"MySQL 오류: {err}")
            
            finally:
                if 'cursor' in locals():
                    cursor.close()
                if conn.is_connected():
                    conn.close()
                    
        except Exception as e:
            self.get_logger().error(f"주문 상태 업데이트 중 오류 발생: {str(e)}")


    def get_current_timestamp(self):
        """현재 시간을 기반으로 타임스탬프 생성"""
        from datetime import datetime
        return datetime.now().strftime("%Y%m%d_%H%M%S")
    
    def run_flask_server(self):
        """Flask 서버를 별도 스레드에서 실행"""
        app = Flask(__name__)
        CORS(app)  # 모든 라우트에 CORS 적용
        
        # Flask 애플리케이션 컨텍스트 내에서 self 참조 유지
        serboway_server = self
        
        @app.route('/order', methods=['POST'])
        def receive_order():
            # 요청에서 JSON 데이터 가져오기
            try:
                data = request.get_json()
                if not data:
                    return jsonify({"message": "데이터가 없습니다"}), 400
                
                serboway_server.get_logger().info(f"주문 수신: {data}")
                
                # 데이터 추출 및 유효성 검사
                table_number = data.get("table_number")
                sandwich = data.get("sandwich")
                sauce = data.get("sauce")
                vegetable = data.get("vegetable")
                cheese = data.get("cheese")
                price = data.get("price")
                
                # 정수형으로 변환
                try:
                    table_number = int(table_number)
                except (ValueError, TypeError):
                    pass

                try:
                    price = int(price)
                except (ValueError, TypeError):
                    pass
                
                # 기본 유효성 검사
                if None in [table_number, sandwich, sauce, vegetable, cheese, price]:
                    missing_fields = [field for field, value in 
                                    zip(["table_number", "sandwich", "sauce", "vegetable", "cheese", "price"],
                                        [table_number, sandwich, sauce, vegetable, cheese, price]) 
                                    if value is None]
                    return jsonify({
                        "message": "필수 필드가 누락되었습니다", 
                        "missing_fields": missing_fields
                    }), 400

                # 데이터베이스에 저장
                conn = serboway_server.get_db_connection()
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
                    serboway_server.get_logger().info(f"주문 #{order_id} 저장 완료")
                    
                    # 주문 데이터에 order_id 추가
                    data['order_id'] = order_id
                    
                    # ROS 토픽으로 주문 정보 발행
                    serboway_server.process_order(data)
                    
                    return jsonify({
                        "message": "주문이 성공적으로 처리되었습니다",
                        "order_id": order_id
                    }), 200
                    
                except mysql.connector.Error as err:
                    serboway_server.get_logger().error(f"MySQL 오류: {err}")
                    return jsonify({"message": "DB 저장 실패", "error": str(err)}), 500
                
                finally:
                    if 'cursor' in locals():
                        cursor.close()
                    if conn.is_connected():
                        conn.close()
                        
            except Exception as e:
                serboway_server.get_logger().error(f"서버 오류: {str(e)}")
                return jsonify({"message": "서버 오류", "error": str(e)}), 500

        @app.route('/health', methods=['GET'])
        def health_check():
            """서버가 실행 중인지 확인하는 간단한 엔드포인트"""
            return jsonify({"status": "ok", "message": "서버가 정상적으로 실행 중입니다"}), 200
        
        # Flask 서버 실행
        serboway_server.get_logger().info("Flask 서버 시작 중...")
        app.run(host="0.0.0.0", port=5003)
        serboway_server.get_logger().info("Flask 서버 종료")

def main(args=None):
    rclpy.init(args=args)
    
    node = SerbowayControlServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)  # 모든 스레드 종료를 위해 프로세스 종료

if __name__ == '__main__':
    main()