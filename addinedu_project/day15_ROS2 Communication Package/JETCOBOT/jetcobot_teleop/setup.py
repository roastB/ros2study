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
    entry_points={
        'console_scripts': [
            'teleop_node = jetcobot_teleop.teleop_node:main',  # teleop_node 실행 가능하도록 설정
        ],
    },
    zip_safe=True,
)
