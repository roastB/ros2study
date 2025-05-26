from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 주문 버퍼 노드
        Node(
            package='serboway_system',
            executable='order_buffer',
            name='order_buffer',
            output='screen',
        ),
        
        # 주문 처리 노드
        Node(
            package='serboway_system',
            executable='order_process',
            name='order_buffer_checker',
            output='screen',
        ),
    ])