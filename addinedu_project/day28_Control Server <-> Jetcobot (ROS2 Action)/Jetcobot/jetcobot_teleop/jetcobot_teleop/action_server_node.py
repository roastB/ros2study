import rclpy
from rclpy.node import Node
from pymycobot.mycobot280 import MyCobot280
# 수정된 import 경로 - 빌드 후 생성되는 실제 경로
from jetcobot_teleop.action import OrderJetcobot
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

import time

# 클래스 이름을 JetCobotActionServer로 변경하여 충돌 방지
class JetCobotActionServer(Node):
    def __init__(self):
        super().__init__('action_server')

        # MyCobot 연결
        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.thread_lock = True
        self.get_logger().info('[✔] JETCOBOT 로봇이 연결되었습니다.')

        # Action 서버 시작
        self._action_server = ActionServer(
            self,
            OrderJetcobot,
            'order_jetcobot',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'[✔] 주문 수신: {goal_handle.request.order}')

        order = goal_handle.request.order

        # 명령어에 따라 포즈 선택
        if order == 'home':
            angles = [0, 0, 0, 0, 0, 0]
        elif order == 'camera':
            angles = [-90.96, -11.07, -35.36, -38.33, -0.79, -47.02]
        else:
            self.get_logger().warn('[!!] 알 수 없는 명령입니다. 실패 반환')
            goal_handle.abort()
            return OrderJetcobot.Result(success=False)

        speed = 35
        self.mc.send_angles(angles, speed)
        time.sleep(3)

        self.get_logger().info(f'[✔] {order} 동작 완료')
        goal_handle.succeed()

        result = OrderJetcobot.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    # 여기서도.클래스 이름 변경 적용
    node = JetCobotActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()