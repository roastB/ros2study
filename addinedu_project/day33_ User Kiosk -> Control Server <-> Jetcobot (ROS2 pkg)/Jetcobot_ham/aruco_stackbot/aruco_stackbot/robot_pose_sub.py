import rclpy
from rclpy.node import Node
from my_msgs.msg import MarkerPose, MarkerPoseArray
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from aruco_stackbot.robot import get_robot

# 1. 데이터 입력 (각 행: [x, y, z, roll, pitch, yaw])
A_data = np.array([
    [-168.36, 17.96, 545.68, 177.4, -0.09, -2.84],
    [-150.83, -83.0, 554.05, -182.33, -3.67, -7.21],
    [-73.89, -146.27, 546.55, 180.86, 1.37, -5.13],
    [-2.85, -108.42, 555.67, 182.52, 5.19, -10.26],
    [51.45, -152.55, 551.78, -152.7, -6.19, 9.13],
    [98.79, -96.13, 550.06, 179.29, 7.18, -8.69],
    [139.99, -159.72, 551.1, -176.1, -3.57, 6.16],
    [207.88, -109.35, 544.01, -179.19, -2.22, 1.56],
    [190.17, -52.41, 544.65, -179.57, -2.24, 0.99],
    [152.39, 26.93, 551.77, 177.41, -3.59, -4.38],
    [188.31, 93.27, 539.9, -177.38, 0.33, 2.23]
])
B_data = np.array([
    [77.9, 168.0, 118.9, 179.18, 0.01, 136.49],
    [171.8, 147.5, 122.0, 176.51, 0.57, 138.02],
    [239.4, 78.6, 124.9, 179.56, 1.03, 139.38],
    [193.4, 10.4, 123.5, 178.39, 4.45, 134.22],
    [231.0, -40.9, 123.7, 176.47, 7.53, 139.65],
    [180.7, -87.5, 124.4, 179.04, 4.83, 131.26],
    [248.1, -122.7, 131.3, 179.43, 5.99, 138.98],
    [204.8, -187.5, 131.3, -178.17, 5.3, 138.3],
    [143.4, -171.3, 124.8, -179.56, 5.82, 139.17],
    [61.8, -133.7, 120.1, -179.7, 6.24, 140.44],
    [-1.4, -171.5, 121.5, -178.94, 6.16, 138.05]
])
# 2. 위치만 추출
A_pos = A_data[:, :3]
B_pos = B_data[:, :3]

# 3. Kabsch 알고리즘 (Rigid transform: 회전 R, 이동 t)
def rigid_transform_3D(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R_mat = Vt.T @ U.T
    if np.linalg.det(R_mat) < 0:
        Vt[2, :] *= -1
        R_mat = Vt.T @ U.T
    t = centroid_B - R_mat @ centroid_A
    return R_mat, t

R_pos, t_pos = rigid_transform_3D(A_pos, B_pos)

# 4. 오일러 각 → 회전행렬 변환
def euler_to_matrix(angles_deg):
    # angles_deg: [roll, pitch, yaw] (각도, 단위: degree)
    return R.from_euler('xyz', angles_deg, degrees=True).as_matrix()

A_rotmats = np.array([euler_to_matrix(a[3:]) for a in A_data])
B_rotmats = np.array([euler_to_matrix(b[3:]) for b in B_data])

# 5. 상대 회전행렬 구하기 (B = R_ori * A)
relative_rotmats = [B_rotmats[i] @ A_rotmats[i].T for i in range(len(A_rotmats))]
# 평균 회전행렬 구하기 (scipy의 Rotation 평균 사용)
R_ori = R.from_matrix(relative_rotmats).mean().as_matrix()

# 6. 변환 함수
def transform_pose_A_to_B(pos_A, rotmat_A):
    pos_B = R_pos @ pos_A + t_pos
    rotmat_B = R_ori @ rotmat_A
    return pos_B, rotmat_B

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('robot_pose_sub')
        self.subscription = self.create_subscription(
            MarkerPoseArray,
            'marker_pose_array',  # 실제 토픽명으로 변경
            self.listener_callback,
            10)
        # 필요한 데이터 미리 계산
        self.A_rotmats = A_rotmats
        self.R_pos = R_pos
        self.t_pos = t_pos
        self.R_ori = R_ori
        self.processing = False # 중복 호출 방지
        self.gripper_open_size = 100

    def listener_callback(self, msg):
        if self.processing:
            return
        if len(msg.markers) == 0:
            self.get_logger().info("No marker detected")
            return
        
        self.processing = True
        mc = get_robot()

        count = 0
        
        for marker in msg.markers: 
            # marker = msg.markers[0]
            print(marker)
            
            pose6d = marker.pose_of_6d
            if len(pose6d) < 6:
                self.get_logger().warn('pose_of_6d 길이가 6보다 작습니다.')
                continue
            
            pos_A = np.array([pose6d[0], pose6d[1], pose6d[2]])
            # euler_A = np.array([pose6d[3], pose6d[4], pose6d[5]])
            # rotmat_A = euler_to_matrix(euler_A)
            rotmat_A = self.A_rotmats[0]
            pos_B, rotmat_B = transform_pose_A_to_B(pos_A, rotmat_A)
            euler_B = R.from_matrix(rotmat_B).as_euler('xyz', degrees=True)
            print("=== 변환 결과 ===")
            print("입력 위치 (A):", pos_A)
            print("변환된 위치 (B):", pos_B)
            print("변환된 오일러 각(roll, pitch, yaw):", euler_B)

            custom_angles = [0, 45, 0, -45, 0, 135]
            mc.send_angles(custom_angles, 50)
            time.sleep(3)
            print("홈 각도:", custom_angles)

            # 그리퍼 열기
            mc.set_gripper_value(self.gripper_open_size, 50)
            time.sleep(1)

            print("마커 위쪽으로 이동합니다.")
            custom_coords = [pos_B[0], pos_B[1], pos_B[2] + 50, euler_B[0], euler_B[1], euler_B[2]]
            mc.send_coords(custom_coords, 50, 1)
            time.sleep(3)
            print("마커 위 좌표:", custom_coords)

            print("마커로 이동합니다.")
            custom_coords = [pos_B[0], pos_B[1], pos_B[2], euler_B[0], euler_B[1], euler_B[2]]
            mc.send_coords(custom_coords, 50, 1)
            time.sleep(1)
            print("마커 좌표:", custom_coords)

            # 그리퍼 닫기
            mc.set_gripper_value(0, 50)
            time.sleep(1)

            # print("들어올리는 자세.")
            # custom_coords = [pos_B[0], pos_B[1], pos_B[2] + 100, euler_B[0], euler_B[1], euler_B[2]]
            # mc.send_coords(custom_coords, 50, 1)
            # time.sleep(3)
            # print("올라온 좌표:", custom_coords)

            # print("지정한 좌표 위로 이동합니다.") # z = 113.6
            # custom_coords = [244.2, -42.6, 150 + count*100, -179.43, 2.28, 136.13]
            # mc.send_coords(custom_coords, 50, 1)
            # time.sleep(3)
            # print("지정한 좌표 위:", custom_coords)

            # print("지정한 좌표로 이동합니다.")
            # custom_coords = [244.2, -42.6, 130 + count*30, -179.43, 2.28, 136.13]
            # mc.send_coords(custom_coords, 50, 1)
            # time.sleep(3)
            # print("지정한 좌표:", custom_coords)

            # # 그리퍼 열기
            # mc.set_gripper_value(self.gripper_open_size, 50)
            # time.sleep(1)
            
            print("지정한 각도로 이동합니다.")
            custom_angles = [0, 0, 0, 0, 0, 135]
            mc.send_angles(custom_angles, 30)
            time.sleep(3)
            print("지정한 각도:", custom_angles)

            

            count += 1

        custom_angles = [0, 0, 0, -30, 0, 135]
        mc.send_angles(custom_angles, 50)
        time.sleep(3)
        print("홈 각도:", custom_angles)

        self.processing = False

def main(args=None):
    rclpy.init(args=args)
    print("init")
    node = PoseTransformer()
    print("node")
    rclpy.spin(node)
    print("node end")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
