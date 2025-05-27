from flask import Flask, request, jsonify
from flask_cors import CORS  # 🔹 추가
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading

app = Flask(__name__)
CORS(app)  # 🔹 모든 경로에 대해 CORS 허용

class Ros2Publisher(Node):
    def __init__(self):
        super().__init__('flask_ros2_publisher')
        self.pub = self.create_publisher(Bool, 'ui_button', 10)

    def publish_button_press(self, pressed: bool):
        msg = Bool()
        msg.data = pressed
        self.pub.publish(msg)
        self.get_logger().info(f'Published ui_push_button: {pressed}')

rclpy.init()
ros2_publisher_node = Ros2Publisher()

def ros2_spin():
    rclpy.spin(ros2_publisher_node)

spin_thread = threading.Thread(target=ros2_spin, daemon=True)
spin_thread.start()

@app.route('/pickup', methods=['POST'])
def pickup():
    data = request.json or {}
    pressed = data.get('pressed', True)

    ros2_publisher_node.publish_button_press(pressed)

    return jsonify({'status': 'ok', 'pressed': pressed})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5003)
