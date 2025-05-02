from setuptools import setup

package_name = 'jetcobot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'geometry_msgs'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    # 실행할 노드를 추가!
    entry_points={
        'console_scripts': [
            'teleop_node = jetcobot_teleop.teleop_node:main',
            'test_pose_node = jetcobot_teleop.test_pose_node:main',
            'pose_node = jetcobot_teleop.jetcobot_pose_node:main',
            'aruco_detector_node = jetcobot_teleop.aruco_detector_node:main'
        ],
    },
    zip_safe=True,
)
