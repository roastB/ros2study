from setuptools import setup

package_name = 'jetcobot_control_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'mediapipe', 'opencv-python', 'rclpy', 'std_msgs', 'geometry_msgs'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'teleop_publisher = jetcobot_control_teleop.teleop_publisher:main',  # 실행할 노드(teleop_node)를 추가
        ],
    },
    zip_safe=True,
)
