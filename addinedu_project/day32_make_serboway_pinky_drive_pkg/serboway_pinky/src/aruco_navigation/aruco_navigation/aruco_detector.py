#!/usr/bin/env python3
import math
import pickle
import cv2, cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

class ArucoNav(Node):
    def __init__(self):
        super().__init__('aruco_nav')

        # ROS2 파라미터 선언
        self.declare_parameter('calib_path', '/home/addinedu/camera_calibration.pkl')
        self.declare_parameter('camera_id', 2)
        self.declare_parameter('map_width_cm', 200.0)
        self.declare_parameter('map_height_cm', 100.0)
        self.declare_parameter('marker_size_cm', 10.0)

        # 매개변수 가져오기
        CALIB_PATH = self.get_parameter('calib_path').value
        MAP_WIDTH_CM = self.get_parameter('map_width_cm').value
        MAP_HEIGHT_CM = self.get_parameter('map_height_cm').value
        MARKER_SIZE_CM = self.get_parameter('marker_size_cm').value
        camera_id = self.get_parameter('camera_id').value
        
        half = MARKER_SIZE_CM / 2.0

        # 맵 상의 고정 마커 위치 정의(월드 좌표계)
        marker_world = {
            0: (0 + half, 0 + half),  # 왼쪽 상단 마커
            1: (MAP_WIDTH_CM - half, 0 + half),  # 오른쪽 상단 마커
            3: (0 + half, MAP_HEIGHT_CM - half),  # 왼쪽 하단 마커
            4: (MAP_WIDTH_CM - half, MAP_HEIGHT_CM - half),  # 오른쪽 하단 마커
        }

        WAYPOINT_IDS = [6, 7, 8]  # 경유점으로 사용할 마커 ID 목록
        PPM = 5  # 픽셀 당 밀리미터 비율(화면 표시용)
        MAP_W = int(MAP_WIDTH_CM * PPM)  # 시각화할 지도 너비(픽셀)
        MAP_H = int(MAP_HEIGHT_CM * PPM)  # 시각화할 지도 높이(픽셀)

        # 퍼블리셔
        self.pose_pub = self.create_publisher(PointStamped, '/robot_pose', 10)
        self.front_pub = self.create_publisher(PointStamped, '/robot_front', 10)
        self.target_pub = self.create_publisher(Int32, '/target_marker', 10)
        self.wp_pubs = {
            wid: self.create_publisher(PointStamped, f'/waypoint_{wid}', 10)
            for wid in WAYPOINT_IDS
        }
        self.wp_sent = set()
        self.wp_coords = {}
        self.last_target = None
        self.start_pose = None   # 5번 좌표 고정용
        self.cached_H = None
        self.current_pose = None  # 매 프레임 갱신되는 로봇 현재 좌표

        # 카메라 캘리브레이션
        try:
            with open(CALIB_PATH, 'rb') as f:
                calib = pickle.load(f)
            self.mtx, self.dist = calib['camera_matrix'], calib['dist_coeffs']
            self.get_logger().info(f"카메라 캘리브레이션 파일 로드 성공: {CALIB_PATH}")
        except Exception as e:
            self.get_logger().error(f"카메라 캘리브레이션 파일 로드 실패: {e}")
            self.mtx, self.dist = None, None

        # ArUco 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.params = aruco.DetectorParameters()
        self.marker_length = MARKER_SIZE_CM

        # OpenCV 창
        cv2.namedWindow("win", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("win", 640, 480)
        cv2.namedWindow("tf_map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("tf_map", int(MAP_W*0.75), int(MAP_H*0.75))

        try:
            self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().error(f"카메라 {camera_id} 열기 실패")
            else:
                self.get_logger().info(f"카메라 {camera_id} 연결 성공")
        except Exception as e:
            self.get_logger().error(f"카메라 열기 실패: {e}")
            self.cap = None

        self.create_timer(1/30.0, self.cb)

    # 나머지 코드는 기존과 동일하게 유지 (build_homography, pixel_to_world, cb 메서드 등)
    def build_homography(self, corners, ids):
        if ids is None:
            return None
        img_pts, world_pts = [], []
        for i, mid in enumerate(ids.flatten()):
            if mid in marker_world:
                c = corners[i][0]
                img_pts.append([float(c[:,0].mean()), float(c[:,1].mean())])
                world_pts.append(marker_world[mid])
        if len(img_pts) < 4:
            return None
        H, _ = cv2.findHomography(np.array(img_pts,np.float32),
                                  np.array(world_pts,np.float32))
        return H

    def pixel_to_world(self, H, px, py):
        pt = np.array([px, py, 1.0], dtype=np.float32)
        w  = H @ pt; w /= w[2]
        return float(w[0]), float(w[1])

    def cb(self):
        # 기존의 콜백 함수 코드를 그대로 유지
        # 단, 로깅이 필요한 부분에서는 print 대신 self.get_logger().info() 사용
        
        # 생략 - 기존 콜백 코드 그대로 사용

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 리소스 정리
        if hasattr(node, 'cap') and node.cap is not None:
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()