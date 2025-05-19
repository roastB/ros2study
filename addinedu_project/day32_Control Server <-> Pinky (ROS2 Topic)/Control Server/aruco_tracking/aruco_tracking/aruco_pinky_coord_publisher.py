#!/usr/bin/env python3
import math
import pickle
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import traceback

# ===== 설정 =====
CALIB_PATH = '/home/addineud/ros2_ws/src/aruco_tracking/aruco_tracking/config/camera_calibration.pkl'
MAP_WIDTH_CM = 200.0
MAP_HEIGHT_CM = 100.0
MARKER_SIZE_CM = 10.0
half = MARKER_SIZE_CM / 2.0
PPM = 5
MAP_W = int(MAP_WIDTH_CM * PPM)
MAP_H = int(MAP_HEIGHT_CM * PPM)

# 웹캠 인덱스 리스트 (Lenovo FHD Webcam)
CAMERA_INDICES = [2, 3]

marker_world = {
    0: (0 + half, 0 + half),
    1: (MAP_WIDTH_CM - half, 0 + half),
    3: (0 + half, MAP_HEIGHT_CM - half),
    4: (MAP_WIDTH_CM - half, MAP_HEIGHT_CM - half),
}

class ArucoPinkyCoordPublisher(Node):
    def __init__(self):
        super().__init__('aruco_pinky_coord_publisher')
        
        # 로거 설정
        self.get_logger().info("ArUco Pinky Coord Publisher 초기화 시작")
        
        try:
            # 카메라 초기화
            self.get_logger().info("체크1: 카메라 초기화 시작")
            self.cap = self._initialize_camera()
            self.get_logger().info("체크2: 카메라 초기화 완료")
            
            # 실패 시 다른 인덱스 시도
            if self.cap is None:
                for idx in CAMERA_INDICES:
                    if idx != CAMERA_INDICES[0]:
                        self.get_logger().info(f"대체 카메라 인덱스 {idx} 시도")
                        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                        if self.cap.isOpened():
                            break
            
            # 카메라 열기 최종 확인
            if not self.cap or not self.cap.isOpened():
                self.get_logger().error("어떤 카메라도 열 수 없음")
                raise RuntimeError("카메라 초기화 실패")
            
            # 카메라 속성 설정
            self.get_logger().info("체크3: 카메라 속성 설정")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
            
            # 카메라 속성 설정 확인 - 여기에 추가
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.get_logger().info(f"설정된 카메라 해상도: {actual_width}x{actual_height}")

            # 퍼블리셔 설정
            self.get_logger().info("체크4: 퍼블리셔 설정")
            self._setup_publishers()
            
            # 카메라 캘리브레이션 로드
            self.get_logger().info("체크5: 카메라 캘리브레이션 로드")
            self._load_camera_calibration()
            
            # ArUco 설정
            self.get_logger().info("체크6: ArUco 설정")
            self._setup_aruco_detection()
            
            # 프레임 처리 타이머
            self.get_logger().info("체크7: 프레임 처리 타이머 설정")
            self.timer = self.create_timer(1/30.0, self.process_frame)
            
            # OpenCV 창 설정
            # OpenCV 창 설정
            self.get_logger().info("체크8: OpenCV 창 설정")
            cv2.namedWindow("ArUco Detection", cv2.WINDOW_NORMAL)
            # 창 크기를 원본 이미지 크기에 맞게 설정
            cv2.resizeWindow("ArUco Detection", 1280, 960)
        
        except Exception as e:
            self.get_logger().error(f"초기화 중 오류 발생: {e}")
            raise

    def _initialize_camera(self):
        """카메라 초기화"""
        try:
            # 첫 번째 인덱스로 시도
            self.get_logger().info(f"카메라 인덱스 {CAMERA_INDICES[0]} 초기화 시도")
            cap = cv2.VideoCapture(CAMERA_INDICES[0], cv2.CAP_V4L2)
            
            if not cap.isOpened():
                self.get_logger().warn(f"카메라 인덱스 {CAMERA_INDICES[0]} 열기 실패")
                return None
            
            return cap
        except Exception as e:
            self.get_logger().error(f"카메라 초기화 오류: {e}")
            return None

    def _setup_publishers(self):
        """퍼블리셔 설정"""
        # Pinky에게 보낼 퍼블리셔들
        self.pose_pub = self.create_publisher(PointStamped, '/robot_pose', 10)
        self.front_pub = self.create_publisher(PointStamped, '/robot_front', 10)
        self.waypoint_6_pub = self.create_publisher(PointStamped, '/waypoint_6', 10)
        self.waypoint_7_pub = self.create_publisher(PointStamped, '/waypoint_7', 10)

    def _load_camera_calibration(self):
        """카메라 캘리브레이션 로드"""
        # 기본 캘리브레이션 값 설정
        default_mtx = np.array([
            [1000, 0, 640],
            [0, 1000, 360],
            [0, 0, 1]
        ], dtype=np.float32)
        
        default_dist = np.zeros((1, 5), dtype=np.float32)
        
        try:
            with open(CALIB_PATH, 'rb') as f:
                calib = pickle.load(f)
            self.mtx = calib.get('camera_matrix', default_mtx)
            self.dist = calib.get('dist_coeffs', default_dist)
        except FileNotFoundError:
            self.get_logger().warn(f"캘리브레이션 파일 {CALIB_PATH} 찾을 수 없음. 기본값 사용.")
            self.mtx = default_mtx
            self.dist = default_dist

    def _setup_aruco_detection(self):
        """ArUco 마커 감지 설정"""
        try:
            # OpenCV 버전 확인 및 출력
            self.get_logger().info(f"OpenCV 버전: {cv2.__version__}")
            
            # ArUco 사전 설정
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
            
            # OpenCV 버전에 따라 DetectorParameters 생성 방식이 다름
            # 최신 OpenCV 버전(4.7.0+)
            try:
                # 최신 버전
                self.aruco_detector = aruco.ArucoDetector(self.aruco_dict)
                self.get_logger().info("최신 ArUco API 사용")
                self.use_new_api = True
            except AttributeError:
                # 이전 버전
                self.aruco_params = aruco.DetectorParameters()
                self.get_logger().info("이전 ArUco API 사용")
                self.use_new_api = False
                
            self.marker_length = MARKER_SIZE_CM
        except Exception as e:
            self.get_logger().error(f"ArUco 설정 중 오류: {e}")
            self.get_logger().error(traceback.format_exc())

    def _create_point_stamped_msg(self, x, y):
        """PointStamped 메시지 생성"""
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        return msg

    def _build_homography(self, corners, ids):
        """호모그래피 행렬 계산"""
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
        
        try:
            H, _ = cv2.findHomography(
                np.array(img_pts, np.float32),
                np.array(world_pts, np.float32)
            )
            return H
        except Exception as e:
            self.get_logger().error(f"호모그래피 계산 실패: {e}")
            return None

    def _pixel_to_world(self, H, px, py):
        """픽셀 좌표를 월드 좌표로 변환"""
        pt = np.array([px, py, 1.0], dtype=np.float32)
        w = H @ pt
        w /= w[2]
        return float(w[0]), float(w[1])

    def process_frame(self):
        """프레임 처리"""
        try:
            # 프레임 캡처
            self.get_logger().info("체크10: 프레임 캡처 시작")
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("프레임 캡처 실패")
                return
            self.get_logger().info("체크11: 프레임 캡처 성공")

            # 이미지 보정
            # 이미지 보정
            try:
                self.get_logger().info("체크12: 이미지 보정 시작")
                h, w = frame.shape[:2]
                self.get_logger().info(f"체크12-1: 프레임 크기 h={h}, w={w}")
                new_mtx, roi = cv2.getOptimalNewCameraMatrix(
                    self.mtx, self.dist, (w,h), 0, (w,h)  # alpha=0으로 변경
                )
                self.get_logger().info("체크12-2: getOptimalNewCameraMatrix 완료")
                und = cv2.undistort(frame, self.mtx, self.dist, None, new_mtx)
                self.get_logger().info("체크12-3: undistort 완료")
                x, y, w0, h0 = roi
                self.get_logger().info(f"체크12-4: ROI 값 x={x}, y={y}, w0={w0}, h0={h0}")
                # ROI 크롭 제거
                img = und  # 원본 크기 그대로 사용
                self.get_logger().info(f"체크12-6: 원본 이미지 사용 (크기: {img.shape[1]}x{img.shape[0]})")
            except Exception as e:
                self.get_logger().error(f"이미지 보정 실패: {e}")
                return
            self.get_logger().info("체크13: 이미지 보정 완료")

            # ArUco 마커 감지
            # ArUco 마커 감지
            self.get_logger().info("체크14: ArUco 마커 감지 시작")
            try:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                self.get_logger().info("체크14-1: 그레이스케일 변환 완료")

                # 원본 코드 그대로 유지
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, 
                    self.aruco_dict
                )

                self.get_logger().info(f"체크14-2: ArUco 마커 감지 완료, 감지된 마커 수: {0 if ids is None else len(ids)}")
            except Exception as e:
                self.get_logger().error(f"ArUco 마커 감지 중 오류: {e}")
                self.get_logger().error(traceback.format_exc())
                # 예외 발생해도 계속 진행 (기본 이미지 보여주기)
                cv2.imshow("ArUco Detection", img)
                cv2.waitKey(1)
                return

            # 호모그래피 계산
            self.get_logger().info("체크15: 호모그래피 계산 시작")
            H = self._build_homography(corners, ids)
            if H is None:
                self.get_logger().warn("체크15-1: 호모그래피 계산 실패, 충분한 마커가 감지되지 않음")
                cv2.imshow("ArUco Detection", img)
                cv2.waitKey(1)
                return
            self.get_logger().info("체크15-2: 호모그래피 계산 성공")

            # 마커 그리기
            self.get_logger().info("체크16: 마커 그리기 시작")
            try:
                if ids is not None:
                    if hasattr(self, 'use_new_api') and self.use_new_api:
                        # 최신 API
                        img_with_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
                        img = img_with_markers
                    else:
                        # 이전 API
                        aruco.drawDetectedMarkers(img, corners, ids)
                    self.get_logger().info("체크16-1: 마커 그리기 완료")
            except Exception as e:
                self.get_logger().error(f"마커 그리기 중 오류: {e}")
                self.get_logger().error(traceback.format_exc())

            # 5번 마커(로봇) 추적
            self.get_logger().info("체크17: 5번 마커(로봇) 추적 시작")
            try:
                robot_marker_index = list(ids.flatten()).index(5)
                self.get_logger().info(f"체크17-1: 5번 마커 발견, 인덱스: {robot_marker_index}")
                robot_corners = corners[robot_marker_index][0]
                
                # 로봇 중심점 계산
                px, py = float(robot_corners[:,0].mean()), float(robot_corners[:,1].mean())
                self.get_logger().info(f"체크17-2: 로봇 픽셀 좌표: px={px:.2f}, py={py:.2f}")
                wx_r, wy_r = self._pixel_to_world(H, px, py)
                self.get_logger().info(f"체크17-3: 로봇 월드 좌표: wx_r={wx_r:.2f}, wy_r={wy_r:.2f}")

                # 로봇 포즈 메시지 발행
                pose_msg = self._create_point_stamped_msg(wx_r, wy_r)
                self.pose_pub.publish(pose_msg)
                self.get_logger().info(f"체크17-4: 로봇 위치 발행 완료")
                
                self.get_logger().info(f"로봇 위치: x={wx_r:.2f}, y={wy_r:.2f}")
            except ValueError as e:
                # 5번 마커가 없는 경우
                self.get_logger().warn(f"체크17-X: 5번 마커 찾기 실패: {e}")
            except Exception as e:
                self.get_logger().error(f"체크17-E: 로봇 마커 처리 중 오류: {e}")

            # 웨이포인트 발행 (6, 7번 마커)
            self.get_logger().info("체크18: 웨이포인트 마커 처리 시작")
            marker_ids = ids.flatten() if ids is not None else []
            for marker_id in [6, 7]:
                try:
                    marker_index = list(marker_ids).index(marker_id)
                    self.get_logger().info(f"체크18-1: {marker_id}번 마커 발견, 인덱스: {marker_index}")
                    marker_corners = corners[marker_index][0]
                    
                    # 마커 중심점 계산
                    px, py = float(marker_corners[:,0].mean()), float(marker_corners[:,1].mean())
                    self.get_logger().info(f"체크18-2: {marker_id}번 마커 픽셀 좌표: px={px:.2f}, py={py:.2f}")
                    wx, wy = self._pixel_to_world(H, px, py)
                    self.get_logger().info(f"체크18-3: {marker_id}번 마커 월드 좌표: wx={wx:.2f}, wy={wy:.2f}")

                    # 웨이포인트 메시지 발행
                    wp_msg = self._create_point_stamped_msg(wx, wy)
                    if marker_id == 6:
                        self.waypoint_6_pub.publish(wp_msg)
                        self.get_logger().info("체크18-4: 웨이포인트 6 발행 완료")
                    else:
                        self.waypoint_7_pub.publish(wp_msg)
                        self.get_logger().info("체크18-4: 웨이포인트 7 발행 완료")
                except ValueError as e:
                    # 해당 마커가 없는 경우
                    self.get_logger().warn(f"체크18-X: {marker_id}번 마커 찾기 실패: {e}")
                except Exception as e:
                    self.get_logger().error(f"체크18-E: {marker_id}번 마커 처리 중 오류: {e}")

            # 이미지 표시
            # 이미지 표시 부분 (process_frame 메서드에서)
            self.get_logger().info("체크19: 이미지 표시 시작")
            # 이미지를 화면에 맞게 축소
            display_img = cv2.resize(img, (640, 480))  # 화면에 맞는 적절한 크기로 축소
            cv2.imshow("ArUco Detection", display_img)
            cv2.waitKey(1)
            self.get_logger().info("체크20: 이미지 표시 완료")
        
        except Exception as e:
            self.get_logger().error(f"프레임 처리 중 오류 발생: {e}")
            # 스택 트레이스 표시
            self.get_logger().error(f"오류 상세 정보: {traceback.format_exc()}")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ArucoPinkyCoordPublisher()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            # 리소스 정리
            if hasattr(node, 'cap') and node.cap:
                node.cap.release()
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"주 프로그램 오류: {e}")

if __name__ == '__main__':
    main()