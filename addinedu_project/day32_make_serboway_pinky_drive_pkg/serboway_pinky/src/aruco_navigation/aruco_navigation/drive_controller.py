#!/usr/bin/env python3
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        
        # ROS2 파라미터 선언
        self.declare_parameter('fixed_omega', 0.1)
        self.declare_parameter('turn_interval', 0.05)
        self.declare_parameter('pause_interval', 0.07)
        self.declare_parameter('drive_speed', 0.1)
        self.declare_parameter('angle_tolerance', 5.0)
        self.declare_parameter('dist_tolerance', 1.0)
        self.declare_parameter('obs_detect_thresh', 0.15)
        self.declare_parameter('obs_clear_thresh', 0.25)
        self.declare_parameter('side_detect_thresh', 0.15)
        self.declare_parameter('drive_obs_time', 0.3)
        self.declare_parameter('surf_drive_time', 0.3)
        self.declare_parameter('too_close_thresh', 0.10)
        
        # 파라미터 값 가져오기
        self.fixed_omega = self.get_parameter('fixed_omega').value
        self.turn_interval = self.get_parameter('turn_interval').value
        self.pause_interval = self.get_parameter('pause_interval').value
        self.drive_speed = self.get_parameter('drive_speed').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.dist_tolerance = self.get_parameter('dist_tolerance').value
        self.obs_detect_thresh = self.get_parameter('obs_detect_thresh').value
        self.obs_clear_thresh = self.get_parameter('obs_clear_thresh').value
        self.side_detect_thresh = self.get_parameter('side_detect_thresh').value
        self.drive_obs_time = self.get_parameter('drive_obs_time').value
        self.surf_drive_time = self.get_parameter('surf_drive_time').value
        self.too_close_thresh = self.get_parameter('too_close_thresh').value
        
        # subscriptions
        self.create_subscription(PointStamped, '/robot_pose', self.pose_cb, 10)
        self.create_subscription(PointStamped, '/robot_front', self.front_cb, 10)
        for wid in (6, 7, 8):
            self.create_subscription(
                PointStamped, f'/waypoint_{wid}',
                lambda msg, wid=wid: self.wp_cb(wid, msg),
                10
            )
        self.create_subscription(Int32, '/target_marker', self.target_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 상태 변수 초기화
        self.wx_r = self.wy_r = None
        self.wx_f = self.wy_f = None
        self.marker_world = {}
        self.target = None
        self.start_pose = None
        self.last_target = None
        self.d_f = self.d_l = self.d_r = float('nan')

        # FSM 상태
        self.state = 'IDLE'
        self.rotation_dir = 0.0
        self.accum_time = 0.0
        self.prev_time = time.time()
        self._obs_heading0 = 0.0

        # 제어 루프 타이머 (200Hz)
        self.create_timer(1/200.0, self.control_loop)
        
        self.get_logger().info("DriveNode 초기화 완료")

    # 콜백 메서드들
    def pose_cb(self, msg: PointStamped):
        self.wx_r, self.wy_r = msg.point.x, msg.point.y

    def front_cb(self, msg: PointStamped):
        self.wx_f, self.wy_f = msg.point.x, msg.point.y

    def wp_cb(self, wid, msg: PointStamped):
        self.marker_world[wid] = (msg.point.x, msg.point.y)
        self.get_logger().info(f"웨이포인트 {wid} 위치 수신: ({msg.point.x:.2f}, {msg.point.y:.2f})")

    def target_cb(self, msg: Int32):
        tid = msg.data
        if tid in self.marker_world and self.wx_r is not None:
            self.start_pose = (self.wx_r, self.wy_r)
            self.last_target = tid
            self.target = self.marker_world[tid]
            self.state = 'IDLE'
            self.accum_time = 0.0
            # 이전 플래그 제거
            for a in ('drive_obs_cleared', 'surf_obs_detected', 'surf_clear_time',
                      '_rotate_obs_start', '_rotate_surf_start', '_rotate3_start', 'third_obs_detected'):
                if hasattr(self, a):
                    delattr(self, a)
            self.get_logger().info(f"▶ 목표 설정: {tid} ({self.target[0]:.2f}, {self.target[1]:.2f})")

    def scan_cb(self, msg: LaserScan):
        r = np.array(msg.ranges)
        N = len(r); mid = N//2
        front_idx = (mid + N//2) % N
        left_idx = (mid - N//4) % N
        right_idx = (mid + N//4) % N
        span = int(math.radians(5)/msg.angle_increment)
        
        def md(idx):
            w = np.concatenate([r[idx-span:idx], r[idx:idx+span+1]])
            v = w[(w>0)&~np.isinf(w)]
            return float(v.min()) if v.size else float('nan')
            
        self.d_f, self.d_l, self.d_r = md(front_idx), md(left_idx), md(right_idx)

    def control_loop(self):
        # 제어 루프 코드는 기존과 동일하게 유지
        # 로깅이 필요한 부분은 self.get_logger().info() 사용
        
        # 생략 - 기존 제어 루프 코드 그대로 사용

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()