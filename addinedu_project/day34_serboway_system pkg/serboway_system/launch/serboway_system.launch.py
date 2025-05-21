from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 컨트롤 서버 노드
        Node(
            package='serboway_system',
            executable='control_server',
            name='serboway_control_server',
            output='screen'
        ),
        
        # 주문 관리 노드들
        Node(
            package='serboway_system',
            executable='db_order_manager',
            name='db_order_manager',
            output='screen'
        ),
        Node(
            package='serboway_system',
            executable='order_buffer',
            name='order_buffer',
            output='screen'
        ),
        
        # 작업 할당 노드
        Node(
            package='serboway_system',
            executable='task_allocator',
            name='task_allocator',
            output='screen'
        ),
    ])