import rclpy
from rclpy.node import Node
from jetcobot_interfaces.action import OrderJetcobot
from rclpy.action import ActionClient

class OrderClient(Node):
    def __init__(self):
        super().__init__('order_client')
        self._action_client = ActionClient(self, OrderJetcobot, 'order')

    def send_goal(self, order):
        goal_msg = OrderJetcobot.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: {order}')
        self._action_client.wait_for_server()

        # Goal을 서버로 전송하고, 피드백 콜백 설정
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # 목표 응답에 대한 콜백 설정
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('[✗] Goal rejected')
            return
            
        self.get_logger().info('[✓] Goal accepted')
        
        # 결과를 가져오기 위한 future 설정
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: {result.status}')
        # 작업 완료 후 종료
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback received: {feedback.progress}%')

def main(args=None):
    rclpy.init(args=args)
    client = OrderClient()
    client.send_goal('[!] Make a sandwich')
    rclpy.spin(client)

if __name__ == '__main__':
    main()