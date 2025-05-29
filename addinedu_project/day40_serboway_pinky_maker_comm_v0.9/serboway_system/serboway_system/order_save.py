# serboway_system/order_save.py

import rclpy
from rclpy.node import Node
import json
import pymysql
import logging

from serboway_system.order_buffer import order_buffer_instance
from my_msgs.srv import CheckOrderBuffer, PopOrderBuffer

class DBOrderManager:
    def __init__(self, host='localhost', user='serboway', password='4011', db='serboway_db'):
        try:
            self.conn = pymysql.connect(
                host=host,
                user=user,
                password=password,
                database=db,
                charset='utf8mb4',
                cursorclass=pymysql.cursors.DictCursor
            )
            self.cursor = self.conn.cursor()
            logging.info('MySQL connection established.')
        except Exception as e:
            logging.error(f"DB connection error: {e}")
            self.conn = None

    def insert_order(self, order_dict):
        """주문을 데이터베이스에 삽입"""
        if self.conn is None:
            logging.error("DB connection not established.")
            return False

        sql = """
        INSERT INTO orders (table_number, sandwich, sauce, vegetable, cheese, price, status)
        VALUES (%s, %s, %s, %s, %s, %s, %s)
        """
        try:
            self.cursor.execute(sql, (
                order_dict.get('table_number'),
                order_dict.get('sandwich'),
                order_dict.get('sauce'),
                order_dict.get('vegetable'),
                order_dict.get('cheese'),
                order_dict.get('price'),
                order_dict.get('status', 'pending')  # 기본값 'pending'
            ))
            self.conn.commit()
            
            # 삽입된 주문의 ID 가져오기
            order_id = self.cursor.lastrowid
            logging.info(f"Order inserted into DB with ID: {order_id}")
            return order_id
            
        except Exception as e:
            logging.error(f"Failed to insert order: {e}")
            self.conn.rollback()
            return False

    def update_order_status(self, order_id, status):
        """주문 상태 업데이트 (완료 처리용)"""
        if self.conn is None:
            logging.error("DB connection not established.")
            return False

        sql = "UPDATE orders SET status = %s WHERE id = %s"
        try:
            self.cursor.execute(sql, (status, order_id))
            self.conn.commit()
            logging.info(f"Order {order_id} status updated to {status}")
            return True
        except Exception as e:
            logging.error(f"Failed to update order status: {e}")
            self.conn.rollback()
            return False

    def get_order_by_id(self, order_id):
        """ID로 주문 정보 조회"""
        if self.conn is None:
            logging.error("DB connection not established.")
            return None

        sql = "SELECT * FROM orders WHERE id = %s"
        try:
            self.cursor.execute(sql, (order_id,))
            return self.cursor.fetchone()
        except Exception as e:
            logging.error(f"Failed to get order: {e}")
            return None

    def close(self):
        """데이터베이스 연결 종료"""
        if self.conn:
            self.cursor.close()
            self.conn.close()
            logging.info("DB connection closed.")

class OrderBufferNode(Node):
    def __init__(self):
        super().__init__('order_save')
        self.order_buffer = order_buffer_instance
        self.db_manager = DBOrderManager()
        
        # 로깅 설정
        logging.basicConfig(level=logging.INFO)

        # [※] JSON 파일 경로 (추후 User GUI에서 json 파일 받아오게끔 수정 필요!)
        json_path = '/home/addineud/test_json/order_test.json'

        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                order = json.load(f)
                self.get_logger().info('[✔] JSON load 성공')

            # DB에 주문 삽입 및 ID 받기
            order_id = self.db_manager.insert_order(order)
            if order_id:
                # 주문에 ID 추가
                order['id'] = order_id
                
                # 버퍼에 주문 추가 (테스트용)
                self.order_buffer.enqueue_order(order)
                self.get_logger().info(f'[✔] 주문 저장 완료 (DB ID: {order_id}, Queue 추가)')

                # 서비스 서버들 생성
                self.create_services()

            else:
                self.get_logger().error('[✘] DB 저장 실패. 주문을 버퍼에 저장하지 않습니다.')

        except FileNotFoundError:
            self.get_logger().error(f'[✘] JSON 파일을 찾을 수 없습니다: {json_path}')
            # JSON 파일이 없어도 서비스는 생성
            self.create_services()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'[✘] JSON 파일 형식 오류: {e}')
            self.create_services()
            
        except Exception as e:
            self.get_logger().error(f'[✘] 예상치 못한 오류: {e}')
            self.create_services()

    def create_services(self):
        """ROS2 서비스들 생성"""
        # CheckOrderBuffer 서비스 (주문 수 확인)
        self.check_srv = self.create_service(
            CheckOrderBuffer, 
            'check_order_buffer', 
            self.handle_check_order_buffer
        )
        
        # PopOrderBuffer 서비스 (주문 제거)
        self.pop_srv = self.create_service(
            PopOrderBuffer,
            'pop_order_buffer', 
            self.handle_pop_order_buffer
        )
        
        self.get_logger().info('[✔] CheckOrderBuffer, PopOrderBuffer 서비스 활성화')
        
        # 현재 버퍼 상태 출력
        orders = self.order_buffer.peek_all_orders()
        self.get_logger().info(f'[📊] 현재 버퍼 상태: {len(orders)}개 주문 대기 중')

    def handle_check_order_buffer(self, request, response):
        """주문 버퍼 상태 확인 서비스"""
        orders = self.order_buffer.peek_all_orders()
        response.order_count = len(orders)
        response.is_collectable = (response.order_count == 0)
        
        # 디버그 로그 (필요시 주석 해제)
        # self.get_logger().info(f'[✔] 버퍼 상태 확인 → 현재 주문 수: {response.order_count}개')
        
        return response

    def handle_pop_order_buffer(self, request, response):
        """주문 제거 서비스"""
        try:
            # 버퍼에서 가장 오래된 주문 제거
            removed_order = self.order_buffer.dequeue_order()
            
            if removed_order is not None:
                # DB에서도 상태 업데이트 (완료 처리)
                order_id = removed_order.get('id')
                if order_id:
                    self.db_manager.update_order_status(order_id, 'completed')
                
                # 성공 응답
                response.success = True
                table_num = removed_order.get('table_number', 'unknown')
                sandwich_type = removed_order.get('sandwich', 'unknown')
                response.message = f"주문 제거 완료 (테이블 {table_num}, {sandwich_type})"
                
                # 남은 주문 수 확인
                remaining_orders = self.order_buffer.peek_all_orders()
                response.remaining_orders = len(remaining_orders)
                
                self.get_logger().info(f'[✔] 주문 제거 완료: ID {order_id}, 테이블 {table_num}')
                self.get_logger().info(f'[📊] 남은 주문: {response.remaining_orders}개')
                
            else:
                # 제거할 주문이 없음
                response.success = False
                response.message = "제거할 주문이 없습니다"
                response.remaining_orders = 0
                
                self.get_logger().warn('[⚠️] 제거할 주문이 없음 - 버퍼가 비어있습니다')
                
        except Exception as e:
            # 오류 발생
            response.success = False
            response.message = f"주문 제거 중 오류: {str(e)}"
            
            # 현재 주문 수 확인 (오류 시에도)
            try:
                remaining_orders = self.order_buffer.peek_all_orders()
                response.remaining_orders = len(remaining_orders)
            except:
                response.remaining_orders = 0
            
            self.get_logger().error(f'[❌] 주문 제거 오류: {e}')
        
        return response

    def add_test_orders(self):
        """테스트용 추가 주문들 생성"""
        test_orders = [
            {
                "table_number": 1,
                "sandwich": "BLT",
                "sauce": "mayo",
                "vegetable": "lettuce,tomato",
                "cheese": "cheddar",
                "price": 12000,
                "status": "pending"
            },
            {
                "table_number": 2,
                "sandwich": "Turkey",
                "sauce": "mustard", 
                "vegetable": "lettuce,onion",
                "cheese": "swiss",
                "price": 13000,
                "status": "pending"
            },
            {
                "table_number": 3,
                "sandwich": "Ham",
                "sauce": "mayo",
                "vegetable": "lettuce,pickle",
                "cheese": "american",
                "price": 11000,
                "status": "pending"
            }
        ]
        
        self.get_logger().info('[...] 테스트 주문들 추가 중...')
        
        for i, order in enumerate(test_orders, 1):
            order_id = self.db_manager.insert_order(order)
            if order_id:
                order['id'] = order_id
                self.order_buffer.enqueue_order(order)
                self.get_logger().info(f'[✔] 테스트 주문 {i} 추가: 테이블 {order["table_number"]}, {order["sandwich"]}')
            else:
                self.get_logger().error(f'[✘] 테스트 주문 {i} DB 저장 실패')
        
        # 최종 상태 출력
        orders = self.order_buffer.peek_all_orders()
        self.get_logger().info(f'[📊] 테스트 주문 추가 완료: 총 {len(orders)}개 주문 대기 중')

    def get_buffer_status(self):
        """현재 버퍼 상태 반환"""
        orders = self.order_buffer.peek_all_orders()
        return {
            'count': len(orders),
            'orders': orders
        }

def main(args=None):
    rclpy.init(args=args)
    order_save_node = OrderBufferNode()
    
    # 테스트용 추가 주문 생성 (필요시 주석 해제)
    # order_save_node.add_test_orders()
    
    try:
        order_save_node.get_logger().info('[...] 주문 관리 서비스 시작: PopOrderBuffer 대기 중')
        rclpy.spin(order_save_node)
    except KeyboardInterrupt:
        order_save_node.get_logger().info('[🛑] 주문 관리 서비스 종료 신호 수신')
    finally:
        order_save_node.get_logger().info('[🔄] 리소스 정리 중...')
        order_save_node.db_manager.close()
        order_save_node.destroy_node()
        rclpy.shutdown()
        print('주문 관리 서비스가 완전히 종료되었습니다.')

if __name__ == '__main__':
    main()