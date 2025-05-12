import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pymycobot.mycobot280 import MyCobot280

class JetCobotFollowerNode(Node):
    def __init__(self):
        super().__init__('jetcobot_control_node')

        self.subscription = self.create_subscription(
            Pose,
            'aruco_pose',
            self.pose_callback,
            10
        )

        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.get_logger().info('[✔] JetCobot 연결됨')

        self.speed = 35
        self.last_position = [0.0, 0.0, 0.0]

    def pose_callback(self, msg: Pose):
        x = msg.position.x  # mm
        y = msg.position.y
        z = msg.position.z

        # 너무 작은 변화 무시 (노이즈 필터)
        dx = abs(x - self.last_position[0])
        dy = abs(y - self.last_position[1])
        dz = abs(z - self.last_position[2])

        if dx < 10 and dy < 10 and dz < 10:
            return

        self.get_logger().info(f'[→] 따라가기: x={x:.1f}, y={y:.1f}, z={z:.1f}')

        # JetCobot으로 이동 명령
        self.mc.send_coords([x, y, z, 0, 0, 0], self.speed, 0)
        self.last_position = [x, y, z]

def main(args=None):
    rclpy.init(args=args)
    node = JetCobotFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
