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

        # JSON 파일 경로 (기존 코드)
        json_path = '/home/addineud/test_json/order_test.json'

        try:
            with open(json_path, 'r') as f:
                order = json.load(f)
            self.get_logger().info('JSON 주문 파일 불러오기 성공.')
        except Exception as e:
            self.get_logger().error(f'JSON 파일 읽기 실패: {e}')
            return

        if self.db_manager.insert_order(order):
            self.get_logger().info('DB 저장 성공. 주문을 버퍼에 저장합니다.')
            self.order_buffer.enqueue_order(order) #order_empty를 테스트 할때는 주석처리하기!
        else:
            self.get_logger().error('DB 저장 실패. 주문을 버퍼에 저장하지 않습니다.')

        # [Test용] 5초 간격 주문 버퍼 상태 출력 타이머
        # self.create_timer(10.0, self.show_buffer_state)

        # ROS2 서비스 서버 생성
        self.srv = self.create_service(CheckOrderBuffer, 'check_order_buffer', self.handle_check_order_buffer)
        self.get_logger().info('CheckOrderBuffer 서비스 서버가 시작되었습니다.')

    #def show_buffer_state(self):
    #    order_count = len(self.order_buffer.peek_all_orders())
    #    self.get_logger().info(f'[주문 버퍼 상태] 현재 주문 수: {order_count}')

    def handle_check_order_buffer(self, request, response):
        orders = self.order_buffer.peek_all_orders()
        response.order_count = len(orders)
        response.is_collectable = (len(orders) == 0)  # ✅ 주문 없으면 True
        self.get_logger().info(f'Service 요청 처리됨 - 현재 주문 수: {response.order_count}')
        return response

    def destroy_node(self):
        self.db_manager.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OrderBufferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()