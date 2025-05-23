#!/usr/bin/env python3
"""
업데이트된 Pinky 할당 알고리즘
- 버퍼에서 가장 오래된 주문 1개를 처리
- w(waiting) 또는 r(return) 상태의 Pinky에게 태스크 할당
"""

import rclpy
from rclpy.node import Node
from my_msgs.msg import OrderInformation, CompleteOrder
from std_msgs.msg import String, Bool
import threading
import queue

class PinkyAllocator(Node):
    """
    Pinky 할당을 담당하는 노드
    """
    def __init__(self):
        super().__init__('pinky_allocator_node')
        
        # Pinky 상태 추적
        self.pinky_status = {}  # pinky_id: status (s1, s2, c1, c2, w, r)
        self.pinky_locations = {}  # pinky_id: (x, y) 위치 - 위치 정보가 있을 경우 사용
        
        # 주문 대기열
        self.order_queue = queue.Queue()
        
        # 현재 처리 중인 주문 추적
        self.current_order = None
        self.processing_order = False
        
        # [3] 'w' 또는 'r' 상태의 Pinky에게만 작업 할당 (task_pub)
        self.task_pub = self.create_publisher(
            OrderInformation,
            'pinky_task_assignment',
            10
        )

        # [1] Pinky 상태 요청 메시지 발행 (pinky_status_pub)
        self.pinky_status_pub = self.create_publisher(
            String,
            'pinky_status_request',
            10
        )
        
        # [0] 버퍼에서 가장 오래된 주문 하나 구독 (order_sub)
        self.order_sub = self.create_subscription(
            OrderInformation,
            'new_order',
            self.order_callback,
            10
        )
    
        # [2] Pinky 상태 메시지 구독 (pinky_status_sub)
        self.pinky_status_sub = self.create_subscription(
            String,
            'pinky_status',  # Format: "pinky_id:status"
            self.pinky_status_callback,
            10
        )
        
        # 주기적 처리 타이머
        self.process_timer = self.create_timer(
            1.0,  # 1초마다 실행
            self.process_next_order
        )
        
        # 상태 요청 타이머
        self.status_request_timer = self.create_timer(
            5.0,  # 5초마다 실행
            self.request_pinky_status
        )
        
        # 로깅
        self.get_logger().info('Pinky Allocator 초기화 완료')
    
    def order_callback(self, msg):
        """새로운 주문 처리"""
        self.get_logger().info(f'새 주문 접수: {msg.id}')
        # 주문 정보를 대기열에 추가
        self.order_queue.put(msg)
    
    def pinky_status_callback(self, msg):
        """Pinky 상태 업데이트"""
        try:
            # 메시지 형식: "pinky_id:status[:x:y]" (위치 정보는 선택적)
            parts = msg.data.split(':')
            if len(parts) >= 2:
                pinky_id = parts[0]
                status = parts[1]
                self.pinky_status[pinky_id] = status
                
                # 위치 정보가 있으면 업데이트 (향후 확장 가능성을 위해 유지)
                if len(parts) >= 4:
                    x = float(parts[2])
                    y = float(parts[3])
                    self.pinky_locations[pinky_id] = (x, y)
                    
                self.get_logger().debug(f'{pinky_id} 상태 업데이트: {status}')
                
                # 현재 주문 처리 중이면, 상태 업데이트 시 바로 할당 시도
                if self.processing_order and self.current_order is not None:
                    self.try_allocate_order()
        except Exception as e:
            self.get_logger().error(f'Pinky 상태 메시지 처리 오류: {e}')
    
    def request_pinky_status(self):
        """모든 Pinky에 상태 요청 메시지 발행"""
        status_request = String()
        status_request.data = "status_request"
        self.pinky_status_pub.publish(status_request)
        self.get_logger().debug('Pinky 상태 요청 메시지 발행')
    
    def get_available_pinkies(self):
        """
        'w' (waiting) 또는 'r' (return) 상태의 Pinky 목록 반환
        """
        available_pinkies = [
            pinky_id for pinky_id, status in self.pinky_status.items() 
            if status == 'w' or status == 'r'
        ]
        return available_pinkies
    
    def process_next_order(self):
        """
        대기열에서 다음 주문을 처리
        """
        # 이미 처리 중인 주문이 있으면 종료
        if self.processing_order:
            return
            
        # 대기열이 비어있으면 종료
        if self.order_queue.empty():
            return
            
        # 가장 오래된 주문을 가져옴
        self.current_order = self.order_queue.get()
        self.processing_order = True
        self.get_logger().info(f'주문 {self.current_order.id} 처리 시작')
        
        # Pinky 상태 요청
        self.request_pinky_status()
        
        # 현재 상태 정보로 바로 할당 시도
        self.try_allocate_order()
    
    def try_allocate_order(self):
        """
        현재 주문을 처리 가능한 Pinky에 할당 시도
        """
        if not self.processing_order or self.current_order is None:
            return
            
        # 'w' 또는 'r' 상태의 Pinky 찾기
        available_pinkies = self.get_available_pinkies()
        
        if not available_pinkies:
            self.get_logger().debug('사용 가능한 Pinky가 없습니다. 대기 중...')
            return
            
        # 첫 번째 사용 가능한 Pinky 선택 (추후 최적화 가능)
        selected_pinky = available_pinkies[0]
        
        # 주문을 발행하고 Pinky 상태 업데이트
        self.task_pub.publish(self.current_order)
        
        # Pinky 상태를 's1'(서빙 받으러 가는 중)으로 업데이트
        self.pinky_status[selected_pinky] = 's1'
        
        self.get_logger().info(f'주문 {self.current_order.id}를 {selected_pinky}에 Task 할당했습니다.')
        
        # 처리 완료 표시
        self.current_order = None
        self.processing_order = False

def main(args=None):
    rclpy.init(args=args)
    node = PinkyAllocator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()