import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 필요한 메시지 타입으로 바꿔도 됨

class TeleopSubscriber(Node):
    def __init__(self):
        super().__init__('teleop_subscriber')
        self.subscription = self.create_subscription(
            String,
            'teleop_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f' Received command: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSubscriber()
    node.get_logger().info("🚀 PinkyControllNode started and waiting for commands...")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
