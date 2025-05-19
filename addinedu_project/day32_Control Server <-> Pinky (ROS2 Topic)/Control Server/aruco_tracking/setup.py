from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'aruco_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', ['aruco_tracking/config/camera_calibration.pkl']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python-headless',
        'numpy',
    ],
    zip_safe=True,
    maintainer='addineud',
    maintainer_email='iceative12@gmail.com',
    description='ArUco marker tracking package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_pinky_coord_publisher = aruco_tracking.aruco_pinky_coord_publisher:main',
            'pinky_coord_receiver = aruco_tracking.pinky_coord_receiver:main',
        ],
    },
)