from setuptools import setup
import os
from glob import glob

package_name = 'aruco_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.pkl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addineud',
    maintainer_email='iceative12@gmail.com',
    description='ArUco marker based navigation system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_navigation.aruco_detector:main',
            'drive_controller = aruco_navigation.drive_controller:main',
        ],
    },
)