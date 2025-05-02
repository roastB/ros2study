import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class JetCobotControlNode(Node):
    def __init__(self):
        super().__init__('teleop_subscriber')
        self.subscription = self.create_subscription(
            String,
            'teleop_commands',  # "teleop_commands" 메시지 구독
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)  # 로봇 제어를 위한 Twist 메시지 발행
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received command: "%s"' % msg.data)
        move_cmd = Twist()

        if msg.data == "forward":
            move_cmd.linear.x = 0.2  # 앞으로 이동
        elif msg.data == "backward":
            move_cmd.linear.x = -0.2  # 뒤로 이동
        elif msg.data == "turn_left":
            move_cmd.angular.z = 1.0  # 왼쪽 회전
        elif msg.data == "turn_right":
            move_cmd.angular.z = -1.0  # 오른쪽 회전
        elif msg.data == "stop":
            move_cmd.linear.x = 0.0  # 멈춤
            move_cmd.angular.z = 0.0
        elif msg.data == "circle":
            move_cmd.linear.x = 0.1  # 원형 회전
            move_cmd.angular.z = 0.5
        elif msg.data == "zigzag":
            self.execute_zigzag()  # zigzag 동작 실행
            return  # 퍼블리시 하지 않음, 함수 내에서 퍼블리시 처리
        else:
            move_cmd.linear.x = 0.0  # 멈춤
            move_cmd.angular.z = 0.0

        self.publisher.publish(move_cmd)  # 로봇에게 명령 발행

    def execute_zigzag(self):
        """지그재그 동작 함수"""
        self.get_logger().info("Executing zigzag motion...")
        move_cmd = Twist()
        for i in range(3):
            move_cmd.linear.x = 0.1  # 앞쪽으로 이동
            move_cmd.angular.z = 1.0  # 왼쪽으로 회전
            self.publisher.publish(move_cmd)
            time.sleep(0.5)  # 잠시 대기
            move_cmd.angular.z = -1.0  # 오른쪽으로 회전
            self.publisher.publish(move_cmd)
            time.sleep(0.5)  # 잠시 대기
        move_cmd.linear.x = 0.0  # 멈춤
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)  # 최종 멈춤

def main(args=None):
    rclpy.init(args=args)
    node = JetCobotControlNode()
    node.get_logger().info("🚀 JetCobotControlNode started and waiting for commands...")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
