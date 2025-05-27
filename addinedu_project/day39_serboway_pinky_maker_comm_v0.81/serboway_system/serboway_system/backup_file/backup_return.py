# serboway_system/order_return.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

class OrderReturn(Node):
    def __init__(self):
        super().__init__('order_return')

        # 상태 플래그
        self.command_received = False
        self.task_received = False
        self.serving_started = False
        self.pickup_detect_by_sensor = False
        self.pickup_detect_by_uibutton = False
        
        # Detail 상태 플래그
        self.return_sent = False  # return 명령 중복 방지
        self.latest_command = None # 기존의 Pinky에게 복귀명령을 내려야하기 때문에 표시
        self.last_sensor_state = None

        self.pinky_number = {5, 6, 7}
        self.table_number = {1, 2, 3}

        # Subscribers
        self.create_subscription(Int32, 'pinky_command', self.pinky_command_callback, 10)
        self.create_subscription(Int32, 'pinky_task', self.pinky_task_callback, 10)
        self.create_subscription(Bool, 'proximity_sensor', self.sensor_callback, 10)
        self.create_subscription(Bool, 'ui_button', self.ui_button_callback, 10)

        # Publishers
        self.command_pub = self.create_publisher(Int32, 'pinky_command', 10)
        self.pinky_task_pub = self.create_publisher(Int32, 'pinky_task', 10)

        self.get_logger().info('[✔] order_return node started.')

    def pinky_command_callback(self, msg):
        if msg.data in self.pinky_number:
            self.command_received = True
            self.latest_command = msg.data
            self.get_logger().info(f'pinky_number received: {msg.data}')
            self.check_serving_started()

    def pinky_task_callback(self, msg):
        if msg.data in self.table_number:
            self.task_received = True
            self.get_logger().info(f'pinky_task received: {msg.data}')
            self.check_serving_started()

    def check_serving_started(self):
        if self.command_received and self.task_received and not self.serving_started:
            self.serving_started = True
            self.get_logger().info('Delivery started! Both pinky_command and pinky_task received.')

    def sensor_callback(self, msg):
        if self.serving_started and msg.data:
            if self.last_sensor_state != True:
                self.get_logger().info('[✔] 1.pickup_detect_by_proximity_sensor received')
            self.pickup_detect_by_sensor = True
            self.check_return_condition()
        elif self.serving_started and not msg.data:
            if self.last_sensor_state != False:
                self.get_logger().info('[...] proximity_sensor 대기 중')
        self.last_sensor_state = msg.data

    def ui_button_callback(self, msg):
        if msg.data:
            self.pickup_detect_by_uibutton = True
            self.get_logger().info('[✔] 2.UI push button pressed received')
            self.check_return_condition()
        else:
            self.pickup_detect_by_uibutton = False

    def check_return_condition(self):
        if (self.pickup_detect_by_sensor and self.pickup_detect_by_uibutton and not self.return_sent and self.latest_command is not None):

            self.get_logger().info('[✔] 3. Pickup confirmed. Sending return command...')

            self.command_pub.publish(Int32(data=self.latest_command))
            self.get_logger().info(f'[⇨] Published pinky_command: {self.latest_command}')

            self.pinky_task_pub.publish(Int32(data=0))
            self.get_logger().info('[⇨] Published pinky_task: 0 (return)')

            self.return_sent = True
            self.reset_flags()

    def reset_flags(self):
        self.command_received = False
        self.task_received = False
        self.serving_started = False
        self.pickup_detect_by_sensor = False
        self.pickup_detect_by_uibutton = False
        self.latest_command = None
        self.last_sensor_state = None


def main(args=None):
    rclpy.init(args=args)
    node = OrderReturn()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
