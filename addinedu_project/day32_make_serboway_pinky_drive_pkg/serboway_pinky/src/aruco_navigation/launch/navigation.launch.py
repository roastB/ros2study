from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('aruco_navigation')
    
    # 기본 설정 파일 경로
    default_calib_path = os.path.join(package_dir, 'config', 'camera_calibration.pkl')
    
    # Launch Arguments
    calib_path = LaunchConfiguration('calib_path')
    camera_id = LaunchConfiguration('camera_id')
    drive_speed = LaunchConfiguration('drive_speed')
    
    return LaunchDescription([
        # Launch Arguments 선언
        DeclareLaunchArgument(
            'calib_path',
            default_value=default_calib_path,
            description='카메라 캘리브레이션 파일 경로'
        ),
        
        DeclareLaunchArgument(
            'camera_id',
            default_value='2',
            description='카메라 장치 ID'
        ),
        
        DeclareLaunchArgument(
            'drive_speed',
            default_value='0.1',
            description='로봇 주행 속도 (m/s)'
        ),
        
        # ArUco 마커 감지 노드
        Node(
            package='aruco_navigation',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[{
                'calib_path': calib_path,
                'camera_id': camera_id,
                'map_width_cm': 200.0,
                'map_height_cm': 100.0,
                'marker_size_cm': 10.0
            }]
        ),
        
        # 드라이브 컨트롤러 노드
        Node(
            package='aruco_navigation',
            executable='drive_controller',
            name='drive_controller',
            output='screen',
            parameters=[{
                'fixed_omega': 0.1,
                'turn_interval': 0.05,
                'pause_interval': 0.07,
                'drive_speed': drive_speed,
                'angle_tolerance': 5.0,
                'dist_tolerance': 1.0,
                'obs_detect_thresh': 0.15,
                'obs_clear_thresh': 0.25,
                'side_detect_thresh': 0.15,
                'drive_obs_time': 0.3,
                'surf_drive_time': 0.3,
                'too_close_thresh': 0.10
            }]
        )
    ])