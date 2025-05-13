from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'jetcobot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['jetcobot_teleop', 'jetcobot_teleop.*']),
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
    ],
    entry_points={
        'console_scripts': [
            'order_server = jetcobot_teleop.order_server:main'
        ],
    },
)
