from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
import rclpy

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        # 개별 마커의 위치 발행
        self.publisher_ = self.create_publisher(Pose, 'aruco_pose', 10)
        # 감지된 모든 마커 ID 목록 발행
        self.ids_publisher_ = self.create_publisher(Int32MultiArray, 'detected_aruco_ids', 10)

        # 카메라 열기
        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        if not self.cap.isOpened():
            self.get_logger().error('Camera failed to open...!')
            return
        self.get_logger().info('[✔] Camera opened successfully')

        # ArUco 설정 - OpenCV 4.11.0 버전에 맞게 수정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # 타이머 설정 (30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 최신 OpenCV 버전에 맞게 detectMarkers 호출 방식 수정
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, rejected_img_points = detector.detectMarkers(gray)

        if ids is not None:
            # 감지된 모든 마커 정보 표시
            self.get_logger().info(f'Detected {len(ids)} ArUco markers')
            
            # 감지된 모든 마커 ID 목록 발행
            ids_msg = Int32MultiArray()
            ids_msg.data = [int(id_val[0]) for id_val in ids]
            self.ids_publisher_.publish(ids_msg)
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corner = corners[i][0]
                center = marker_corner.mean(axis=0)
                
                # 마커 정보 로그 출력
                self.get_logger().info(f'  - ArUco ID {marker_id} at x={center[0]:.2f}, y={center[1]:.2f}')
                
                # 각 마커에 대한 위치 정보 발행
                pose_msg = Pose()
                pose_msg.position.x = float(center[0])
                pose_msg.position.y = float(center[1])
                pose_msg.position.z = float(marker_id)  # z값에 마커 ID 저장
                self.publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()