import rclpy
from rclpy.node import Node
import cv2
import os
import time
import datetime
import pickle
from my_msgs.msg import MarkerPose, MarkerPoseArray
import numpy as np

# from scipy.spatial.transform import Rotation as R


class ArucoMarkerPublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_pub')
        self.publisher_ = self.create_publisher(MarkerPoseArray, 'marker_pose_array', 10)
        self.latest_markers = []

    def live_aruco_detection(self, calibration_data, ros_node, publisher):

        try:
            with open('/home/jetcobot/ham_ws/src/aruco_stackbot/resource/camera_calibration.pkl', 'rb') as f:
                calibration_data = pickle.load(f)
            print("Calibration data loaded successfully")
        except FileNotFoundError:
            print("Error: Camera calibration file not found")
            return
        except Exception as e:
            print(f"Error loading calibration data: {e}")
            return
        
        camera_matrix = calibration_data['camera_matrix']
        dist_coeffs = calibration_data['dist_coeffs']

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            
        marker_size = 30  # 35mm
        half_size = marker_size / 2
        marker_3d_edges = np.array([
            [-half_size, -half_size, 0],
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0]
        ], dtype='float32').reshape((4, 1, 3))

        cap = cv2.VideoCapture(0)
    
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        time.sleep(2)

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
            gray_undistorted = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
            
            self.latest_markers = []

            corners, ids, rejected = detector.detectMarkers(gray_undistorted)
            
            if corners :
                for idx, corner in enumerate(corners):
                    #rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
                    corner = np.array(corner).reshape((4, 2))
                    
                    ret, rvec_pnp, tvec_pnp = cv2.solvePnP(
                        marker_3d_edges, 
                        corner, 
                        camera_matrix, 
                        dist_coeffs
                    )

                    marker_id = ids.flatten()
                    rvec = rvec_pnp.flatten()
                    tvec = tvec_pnp.flatten()

                    x = float(round(tvec[0], 2))
                    y = float(round(tvec[1], 2))
                    z = float(round(tvec[2], 2))
                    rx = float(round(np.rad2deg(rvec[0]), 2))
                    ry = float(round(np.rad2deg(rvec[1]), 2))
                    rz = float(round(np.rad2deg(rvec[2]), 2))

                    pose6d = [x, y, z, rx, ry, rz]
                    self.latest_markers.append((int(marker_id[idx]), pose6d))
                
            cv2.imshow("Aruco Detection", frame_undistorted)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('a'):
                self.publish_marker_poses()
                print("'a' 키가 입력되었습니다!")  

    def publish_marker_poses(self):
        msg = MarkerPoseArray()
        for marker_id, pose6d in self.latest_markers:
            marker = MarkerPose()
            marker.id = marker_id
            marker.pose_of_6d = pose6d
            msg.markers.append(marker)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published {len(msg.markers)} marker poses.')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerPublisher()
    try:
        node.live_aruco_detection(None, node, node.publisher_)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

