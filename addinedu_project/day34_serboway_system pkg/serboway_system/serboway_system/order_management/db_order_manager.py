#!/usr/bin/env python3
"""
주문 관리 시스템 노드
- 키오스크에서 주문 수신
- DB에 주문 저장 (현재는 간단한 파일 기반 DB로 구현)
- 주문 버퍼 관리
- 주문 상태 추적 및 업데이트
"""

import rclpy
from rclpy.node import Node
from my_msgs.msg import OrderInformation, CompleteOrder
from std_msgs.msg import String, Int32, Empty, Bool
import queue
import threading
import json
import os
import time
import sqlite3
from datetime import datetime

class OrderManagementSystem(Node):
    """
    주문 관리 시스템 노드
    """
    def __init__(self):
        super().__init__('order_management_node')
        
        # 데이터베이스 초기화
        self.db_path = os.path.expanduser('~/orders.db')
        self.init_database()
        
        # 주문 대기열
        self.order_queue = queue.Queue()
        
        # 주문 상태 관리
        self.order_status = {}  # order_id: status (waiting, processing, completed)
        
        # 대기열에 있는 주문 수
        self.queue_size = 0
        
        # 주문 카운터 (중복 방지)
        self.order_counter = self.get_last_order_id() + 1
        
        # 뮤텍스 (스레드 안전을 위해)
        self.queue_lock = threading.Lock()
        
        # 발행자(Publishers)
        # 주문 하나를 제공하는 토픽 (Pinky 할당기가 구독)
        self.order_pub = self.create_publisher(
            OrderInformation,
            'new_order',
            10
        )
        
        # 주문 상태를 발행하는 토픽
        self.status_pub = self.create_publisher(
            String,
            'order_status',
            10
        )
        
        # 대기열 크기를 발행하는 토픽
        self.queue_size_pub = self.create_publisher(
            Int32,
            'order_queue_size',
            10
        )
        
        # 주문 처리 가능 여부 발행 (true: 처리 가능, false: 처리 불가)
        self.system_status_pub = self.create_publisher(
            Bool,
            'order_system_status',
            10
        )
        
        # 구독자(Subscribers)
        # 키오스크에서 새 주문 접수
        self.kiosk_order_sub = self.create_subscription(
            OrderInformation,
            '/kiosk_order',  # 슬래시 추가
            self.kiosk_order_callback,
            10
        )
        
        # 주문 처리 완료 메시지 수신
        self.order_complete_sub = self.create_subscription(
            Int32,
            'order_complete',
            self.order_complete_callback,
            10
        )
        
        # 주문 상태 조회 요청
        self.status_request_sub = self.create_subscription(
            Int32,
            'order_status_request',
            self.status_request_callback,
            10
        )
        
        # 다음 주문 요청 (pinky_allocator가 처리할 주문 요청)
        self.next_order_sub = self.create_subscription(
            Empty,
            'next_order_request',
            self.next_order_callback,
            10
        )
        
        # 타이머
        # 시스템 상태 발행 타이머
        self.system_status_timer = self.create_timer(
            5.0,  # 5초마다 실행
            self.publish_system_status
        )
        
        # 대기열 상태 발행 타이머
        self.queue_status_timer = self.create_timer(
            3.0,  # 3초마다 실행
            self.publish_queue_status
        )
        
        # 주문 처리 시스템 준비 여부
        self.system_ready = True
        
        # 로깅
        self.get_logger().info('주문 관리 시스템 초기화 완료')
        self.get_logger().info(f'데이터베이스 경로: {self.db_path}')
        self.get_logger().info(f'현재 주문 카운터: {self.order_counter}')
    
    def init_database(self):
        """데이터베이스 초기화"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # 주문 테이블 생성
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS orders (
            id INTEGER PRIMARY KEY,
            order_time TEXT,
            status TEXT,
            ingredients TEXT
        )
        ''')
        
        conn.commit()
        conn.close()
    
    def get_last_order_id(self):
        """마지막 주문 ID 가져오기"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute("SELECT MAX(id) FROM orders")
            result = cursor.fetchone()[0]
            
            conn.close()
            
            return result if result else 0
        except Exception as e:
            self.get_logger().error(f'마지막 주문 ID 조회 오류: {e}')
            return 0
    
    def save_order_to_db(self, order_id, ingredients):
        """주문을 데이터베이스에 저장"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # 현재 시간
            now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # JSON 형태로 재료 목록 저장
            ingredients_json = json.dumps(ingredients)
            
            cursor.execute(
                "INSERT INTO orders (id, order_time, status, ingredients) VALUES (?, ?, ?, ?)",
                (order_id, now, 'waiting', ingredients_json)
            )
            
            conn.commit()
            conn.close()
            
            self.get_logger().info(f'주문 {order_id} DB 저장 완료')
            return True
        except Exception as e:
            self.get_logger().error(f'주문 저장 오류: {e}')
            return False
    
    def update_order_status_in_db(self, order_id, status):
        """데이터베이스에서 주문 상태 업데이트"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute(
                "UPDATE orders SET status = ? WHERE id = ?",
                (status, order_id)
            )
            
            conn.commit()
            conn.close()
            
            self.get_logger().info(f'주문 {order_id} 상태 DB 업데이트: {status}')
            return True
        except Exception as e:
            self.get_logger().error(f'주문 상태 업데이트 오류: {e}')
            return False
    
    def kiosk_order_callback(self, msg):
        """키오스크에서 새 주문이 들어올 때 호출"""
        # 시스템이 준비되지 않았으면 주문 거부
        if not self.system_ready:
            self.get_logger().warn('시스템이 준비되지 않아 주문이 거부되었습니다.')
            return
        
        with self.queue_lock:
            # 주문 ID 확인 (msg에 id가 없거나 0이면 새 ID 할당)
            if not hasattr(msg, 'id') or msg.id == 0:
                order_id = self.order_counter
                self.order_counter += 1
                msg.id = order_id
            else:
                order_id = msg.id
            
            # DB에 주문 저장
            if not self.save_order_to_db(order_id, msg.ingredients):
                self.get_logger().error(f'주문 {order_id} 저장 실패')
                return
            
            # 주문을 대기열에 추가
            self.order_queue.put(msg)
            
            # 주문 상태 업데이트
            self.order_status[order_id] = 'waiting'
            
            # 대기열 크기 업데이트
            self.queue_size = self.order_queue.qsize()
        
        # 로그
        self.get_logger().info(f'새 주문 추가: ID {order_id}, 대기열 크기: {self.queue_size}')
        
        # 상태 발행
        self.publish_order_status(order_id, 'waiting')
        
        # 대기열 크기 발행
        self.publish_queue_size()
    
    def next_order_callback(self, msg):
        """다음 주문 요청을 처리 (Pinky 할당기가 호출)"""
        # 대기열에서 주문 가져오기
        next_order = None
        
        with self.queue_lock:
            if not self.order_queue.empty():
                next_order = self.order_queue.get()
                
                # 주문 상태 업데이트
                self.order_status[next_order.id] = 'processing'
                self.update_order_status_in_db(next_order.id, 'processing')
                
                # 대기열 크기 업데이트
                self.queue_size = self.order_queue.qsize()
        
        # 주문이 있으면 발행
        if next_order:
            self.order_pub.publish(next_order)
            self.get_logger().info(f'주문 {next_order.id} 발행 완료, Pinky 할당 대기 중')
            
            # 상태 발행
            self.publish_order_status(next_order.id, 'processing')
            
            # 대기열 크기 발행
            self.publish_queue_size()
        else:
            self.get_logger().info('대기 중인 주문이 없습니다.')
    
    def order_complete_callback(self, msg):
        """주문 완료 메시지 처리"""
        order_id = msg.data
        
        with self.queue_lock:
            if order_id in self.order_status:
                # 주문 상태 업데이트
                self.order_status[order_id] = 'completed'
                self.update_order_status_in_db(order_id, 'completed')
        
        # 로그
        self.get_logger().info(f'주문 {order_id} 완료 처리')
        
        # 상태 발행
        self.publish_order_status(order_id, 'completed')
    
    def status_request_callback(self, msg):
        """주문 상태 조회 요청 처리"""
        order_id = msg.data
        
        status = 'unknown'
        with self.queue_lock:
            if order_id in self.order_status:
                status = self.order_status[order_id]
        
        # 상태 발행
        self.publish_order_status(order_id, status)
        self.get_logger().info(f'주문 {order_id} 상태 조회: {status}')
    
    def publish_order_status(self, order_id, status):
        """주문 상태를 발행"""
        status_msg = String()
        status_msg.data = f"{order_id}:{status}"
        self.status_pub.publish(status_msg)
    
    def publish_queue_size(self):
        """대기열 크기를 발행"""
        size_msg = Int32()
        size_msg.data = self.queue_size
        self.queue_size_pub.publish(size_msg)
    
    def publish_system_status(self):
        """시스템 상태를 발행"""
        status_msg = Bool()
        status_msg.data = self.system_ready
        self.system_status_pub.publish(status_msg)
        
        # 시스템 가용성 결정 로직 (예: 대기열이 너무 길면 일시적으로 주문 거부)
        with self.queue_lock:
            if self.queue_size > 10:  # 대기열 크기가 10을 초과하면 주문 거부
                self.system_ready = False
            else:
                self.system_ready = True
    
    def publish_queue_status(self):
        """주기적으로 대기열 상태를 발행"""
        self.publish_queue_size()
        
        # 상세 대기열 상태 로그
        with self.queue_lock:
            waiting_count = sum(1 for status in self.order_status.values() if status == 'waiting')
            processing_count = sum(1 for status in self.order_status.values() if status == 'processing')
            completed_count = sum(1 for status in self.order_status.values() if status == 'completed')
        
        self.get_logger().debug(f'대기열 상태: 대기 중 {waiting_count}, 처리 중 {processing_count}, 완료 {completed_count}')

def main(args=None):
    rclpy.init(args=args)
    node = OrderManagementSystem()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()