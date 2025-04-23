import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher') #jetcobot_control_node
        self.publisher_ = self.create_publisher(String, 'teleop_commands', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = '🥪 JETCOBOT Test SerboWay Command'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
