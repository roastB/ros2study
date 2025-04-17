from setuptools import setup

package_name = 'mediapipe_jetcobot_teleop'

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
            'teleop_node = mediapipe_jetcobot_teleop.teleop_node:main',  # 이 부분을 추가
        ],
    },
    zip_safe=True,
)
