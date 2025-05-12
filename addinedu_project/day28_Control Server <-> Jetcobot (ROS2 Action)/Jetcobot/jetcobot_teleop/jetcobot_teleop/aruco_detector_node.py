from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
import rclpy

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.publisher_ = self.create_publisher(Pose, 'aruco_pose', 10)
        self.ids_publisher_ = self.create_publisher(Int32MultiArray, 'detected_aruco_ids', 10)

        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        if not self.cap.isOpened():
            self.get_logger().error('[✕] Camera failed to open...!')
            return
        self.get_logger().info('[✔] Camera opened successfully')

        # ArUco 설정 - 새로운 API 사용
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # === 카메라 캘리브레이션 값 ===
        self.camera_matrix = np.array([
            [965.9166294, 0, 286.74986444],
            [0, 963.60630608, 165.32826283],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array([
            [-3.81770715e-01, -4.12378540e-01, -4.33447987e-04, -1.65533463e-04, 1.79885791e+00]
        ])

        # 마커 한 변의 길이 (단위: meter)
        self.marker_length = 0.035  # 3.5cm

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            self.get_logger().info(f'Detected {len(ids)} ArUco markers')

            ids_msg = Int32MultiArray()
            ids_msg.data = [int(id_val[0]) for id_val in ids]
            self.ids_publisher_.publish(ids_msg)

            # 각 마커에 대해 포즈 추정
            for i in range(len(ids)):
                marker_id = ids[i][0]
                
                # 각 마커 코너에 대한 객체 포인트 생성 (3D)
                objPoints = np.array([
                    [-self.marker_length/2, self.marker_length/2, 0],
                    [self.marker_length/2, self.marker_length/2, 0],
                    [self.marker_length/2, -self.marker_length/2, 0],
                    [-self.marker_length/2, -self.marker_length/2, 0]
                ])
                
                # 코너 포인트 추출 (2D)
                imgPoints = corners[i][0].reshape(4, 2)
                
                # solvePnP를 사용하여 회전 및 이동 벡터 추정
                success, rvec, tvec = cv2.solvePnP(
                    objPoints, imgPoints, self.camera_matrix, self.dist_coeffs
                )
                
                if success:
                    self.get_logger().info(f'  - ID {marker_id}: X={tvec[0][0]:.3f}m, Y={tvec[1][0]:.3f}m, Z={tvec[2][0]:.3f}m')

                    # Pose 메시지 생성 (mm로 변환)
                    pose_msg = Pose()
                    pose_msg.position.x = float(tvec[0][0]) * 1000  # mm로 변환
                    pose_msg.position.y = float(tvec[1][0]) * 1000  # mm로 변환
                    pose_msg.position.z = float(tvec[2][0]) * 1000  # mm로 변환
                    
                    # 회전 벡터를 쿼터니언으로 변환하여 방향 설정
                    rot_mat = np.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvec, rot_mat)
                    
                    # 추가로 회전 정보를 쿼터니언으로 변환하여 pose_msg에 추가할 수 있음
                    # 쿼터니언 변환에는 scipy를 사용하거나 직접 구현할 수 있음
                    # (여기서는 위치 정보만 사용하기 때문에 생략)

                    self.publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()