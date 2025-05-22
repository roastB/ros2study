import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String  # Bool: 상태 구독, String: 명령 퍼블리시

class MakerCommand(Node):
    def __init__(self):
        super().__init__('maker_command')

        # Subscriber: maker_status 토픽에서 bool 메시지 구독
        self.subscription = self.create_subscription(
            Bool,
            'maker_status',
            self.maker_status_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher: make_command 토픽으로 명령 송신
        self.publisher = self.create_publisher(String, 'make_command', 10)

        self.get_logger().info('[maker_command] 노드가 시작되었습니다. maker_status 구독 대기 중...')

    def maker_status_callback(self, msg: Bool):
        if msg.data:
            # maker_status가 True (대기중) 이면 샌드위치 제작 명령 발행
            command_msg = String()
            command_msg.data = "Start Make Sandwich"
            self.publisher.publish(command_msg)
            self.get_logger().info('[maker_command] maker_status: 대기중(True). 샌드위치 제작 명령 발행!')
        else:
            self.get_logger().info('[maker_command] maker_status: 이미 만드는중(False). 대기 중.')

def main(args=None):
    rclpy.init(args=args)
    node = MakerCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 입력으로 노드 종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
