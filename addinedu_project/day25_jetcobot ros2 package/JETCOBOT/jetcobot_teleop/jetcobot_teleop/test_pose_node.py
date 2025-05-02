import rclpy
from rclpy.node import Node
import time
from pymycobot.mycobot280 import MyCobot280

class JetCobotPoseNode(Node):
    def __init__(self):
        super().__init__('jetcobot_pose_node')
        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.thread_lock = True
        self.get_logger().info('JetCobot에 연결되었습니다.')

        self.target_pose = [-100.96, -11.07, -35.36, -38.33, -0.79, -47.02]
        self.home_pose = [-90.96, -11.07, -35.36, -38.33, -0.79, -47.02]
        self.speed = 35

        self.current_target = self.target_pose
        self.timer = self.create_timer(2.5, self.move_to_pose_loop)  # 2.5초마다 반복

    def move_to_pose_loop(self):
        self.mc.send_angles(self.current_target, self.speed)
        self.get_logger().info(f'각도 {self.current_target}로 이동 명령 전송')

        if self.current_target == self.target_pose:
            self.mc.set_gripper_value(100, self.speed)  # 그리퍼 열기
            self.get_logger().info('그리퍼 열기 명령 전송')
            self.current_target = self.home_pose
        else:
            self.mc.set_gripper_value(0, self.speed)  # 그리퍼 닫기
            self.get_logger().info('그리퍼 닫기 명령 전송')
            self.current_target = self.target_pose

        time.sleep(2)
        self.get_logger().info('동작 완료')

def main(args=None):
    rclpy.init(args=args)
    node = JetCobotPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
