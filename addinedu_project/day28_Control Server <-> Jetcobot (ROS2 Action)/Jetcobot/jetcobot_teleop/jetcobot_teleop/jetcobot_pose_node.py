from rclpy.node import Node
from geometry_msgs.msg import Pose
import rclpy

class JetcobotPoseNode(Node):
    def __init__(self):
        super().__init__('jetcobot_pose_node')
        self.subscription = self.create_subscription(
            Pose,
            'aruco_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        marker_id = int(msg.position.z)  # z값에 마커 ID가 저장되어 있음
        self.get_logger().info(f'Received ArUco ID {marker_id} at x={x:.2f}, y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = JetcobotPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
