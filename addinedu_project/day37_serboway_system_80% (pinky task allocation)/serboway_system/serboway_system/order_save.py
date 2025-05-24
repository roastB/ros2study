# serboway_system/order_management/order_save.py

import rclpy
from rclpy.node import Node
import json
import pymysql
import logging

from serboway_system.order_management.order_buffer import order_buffer_instance
from my_msgs.srv import CheckOrderBuffer

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
                order_dict.get('status')
            ))
            self.conn.commit()
            logging.info(f"Order inserted into DB.")
            return True
        except Exception as e:
            logging.error(f"Failed to insert order: {e}")
            self.conn.rollback()
            return False

    def close(self):
        if self.conn:
            self.cursor.close()
            self.conn.close()
            logging.info("DB connection closed.")

class OrderBufferNode(Node):
    def __init__(self):
        super().__init__('order_save')
        self.order_buffer = order_buffer_instance
        self.db_manager = DBOrderManager()

        # [※] JSON 파일 경로 (추후 User GUI에서 json 파일 받아오게끔 수정 필요!)
        json_path = '/home/addineud/test_json/order_test.json'

        try:
            with open(json_path, 'r') as f:
                order = json.load(f)
                self.get_logger().info('[✔] JSON load')

            if self.db_manager.insert_order(order):
                self.order_buffer.enqueue_order(order) # [Test] order_empty를 테스트 할때는 주석처리하기!
                self.get_logger().info('[✔] 주문 저장 완료 (DB & Queue)')

                # 서비스 서버 생성 (클라이언트로부터 버퍼 상태 요청 받으면 handle_check_order_buffer 함수 실행)
                self.srv = self.create_service(CheckOrderBuffer, 'check_order_buffer', self.handle_check_order_buffer)
                self.get_logger().info('[✔] CheckOrderBuffer 서비스 활성')

            else:
                self.get_logger().error('[✘] DB 저장 실패. 주문을 버퍼에 저장하지 않습니다.')

        except Exception as e:
            self.get_logger().error(f'[✘] JSON 파일 읽기 실패: {e}')
            return

    def handle_check_order_buffer(self, request, response):
        orders = self.order_buffer.peek_all_orders()
        response.order_count = len(orders)
        #self.get_logger().info(f'[₩] 클라이언트로부터 버퍼 상태 요청 → 현재 주문 수: {response.order_count}개')
        return response


def main(args=None):
    rclpy.init(args=args)
    order_save_node = OrderBufferNode()
    
    rclpy.spin(order_save_node)
    order_save_node.destroy_node()
    rclpy.shutdown()