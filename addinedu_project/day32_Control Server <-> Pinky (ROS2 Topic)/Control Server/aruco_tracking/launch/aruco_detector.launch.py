from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_tracking',
            executable='aruco_pinky_coord_publisher',
            name='aruco_pinky_coord_publisher_node',
            output='screen'
        ),
        Node(
            package='aruco_tracking',
            executable='pinky_coord_receiver',
            name='pinky_coord_receiver_node',
            output='screen'
        )
    ])