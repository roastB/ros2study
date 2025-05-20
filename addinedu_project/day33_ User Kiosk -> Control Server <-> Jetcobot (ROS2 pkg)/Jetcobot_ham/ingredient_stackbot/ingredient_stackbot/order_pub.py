import rclpy
from rclpy.node import Node
from my_msgs.msg import OrderInformation  

class OrderPublisher(Node):
    def __init__(self):
        super().__init__('order_publisher')
        self.publisher_ = self.create_publisher(OrderInformation, 'control_jetcobot', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.has_published = False

    def timer_callback(self):
        if self.has_published:
            return
        msg = OrderInformation()
        msg.id = 4
        msg.ingredients = ["불고기 샌드위치", "이탈리안", "양상추", "슈레드 치즈"]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published OrderInformation: id={msg.id}, ingredients={msg.ingredients}')
        self.has_published = True

def main(args=None):
    rclpy.init(args=args)
    node = OrderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()