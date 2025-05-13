import rclpy
from rclpy.node import Node
from jetcobot_interfaces.action import OrderJetcobot
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import time  # time 모듈 임포트 추가

class OrderServer(Node):
    def __init__(self):
        super().__init__('order_server')
        self._action_server = ActionServer(self, OrderJetcobot, 'order', self.execute_callback)
        self.get_logger().info(f'[···] Waiting Order ')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing order: {goal_handle.request.order}')
        
        feedback = OrderJetcobot.Feedback()
        feedback.progress = 0  # 이제 정수형으로 사용 가능

        # 5단계로 나눠서 진행 상황을 클라이언트에게 보내는 예시
        for i in range(1, 6):
            feedback.progress = i * 20  # 진행률을 20%씩 증가
            goal_handle.publish_feedback(feedback)  # 피드백 전송
            self.get_logger().info(f'Progress: {feedback.progress}%')
            time.sleep(1)  # 1초 대기 (time.sleep 사용)

        goal_handle.succeed()

        result = OrderJetcobot.Result()
        result.status = '[✓] Order has been completed!'
        return result

def main():
    rclpy.init()
    server = OrderServer()

    executor = MultiThreadedExecutor()
    executor.add_node(server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()