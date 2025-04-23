import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')
        self.publisher_ = self.create_publisher(String, 'teleop_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초 간격으로 메시지 발행

    def timer_callback(self):
        msg = String()
        msg.data = "🥪 PINKY Test SerboWay Command"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
