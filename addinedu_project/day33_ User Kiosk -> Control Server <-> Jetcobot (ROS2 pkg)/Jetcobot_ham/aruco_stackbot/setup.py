from setuptools import find_packages, setup

package_name = 'aruco_stackbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymycobot',],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='jetcobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_pose_pub = aruco_stackbot.aruco_pose_pub:main',
            'robot_pose_sub = aruco_stackbot.robot_pose_sub:main',
        ],
    },
)
