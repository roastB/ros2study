import rclpy
from rclpy.node import Node
import time
import subprocess  # ✅ 추가

from my_msgs.srv import CheckOrderBuffer
from std_msgs.msg import Bool  # 로봇 상태 확인용

class OrderBufferChecker(Node):
    def __init__(self):
        super().__init__('order_buffer_checker')
        self.cli = self.create_client(CheckOrderBuffer, 'check_order_buffer')
        self.robot_state_publisher = self.create_publisher(Bool, 'maker_status', 10)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 서버 대기 중...')

        self.req = CheckOrderBuffer.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            order_count = self.future.result().order_count
            is_collectable = self.future.result().is_collectable
            self.get_logger().info(f'isCollectable: {is_collectable}, 주문 수: {order_count}')

            if is_collectable:
                self.wait_for_orders()
            else:
                self.publish_robot_state(True)

                # ✅ maker_command 노드 실행
                self.get_logger().info('maker_command 노드를 실행합니다...')
                subprocess.Popen(['ros2', 'run', 'serboway_system', 'maker_command'])
        else:
            self.get_logger().error('서비스 호출 실패')

    def wait_for_orders(self):
        start_time = time.time()
        while time.time() - start_time < 30:
            self.get_logger().info('주문이 들어왔는지 확인 중...')
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

            if self.future.result() and not self.future.result().is_collectable:
                self.get_logger().info('주문이 들어왔습니다! 수거모드 해제')
                return
            time.sleep(3)  # 3초마다 확인

        self.get_logger().info('30초 경과 - 수거모드 ON')

    def publish_robot_state(self, is_waiting: bool):
        msg = Bool()
        msg.data = is_waiting  # 대기중: True
        self.robot_state_publisher.publish(msg)
        state_str = '대기 중' if is_waiting else '제작 중'
        self.get_logger().info(f'로봇 상태 확인 메시지 전송됨: {state_str}({msg.data})')

def main(args=None):
    rclpy.init(args=args)
    node = OrderBufferChecker()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
