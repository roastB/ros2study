from setuptools import find_packages, setup

package_name = 'serboway_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/serboway_system.launch.py']),  # 이 줄 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addineud',
    maintainer_email='iceative12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_save = serboway_system.order_management.order_save:main',
            'order_buffer_checker = serboway_system.jetcobot_command.order_buffer_checker:main',
            'maker_command = serboway_system.jetcobot_command.maker_command:main',
            'control_server = serboway_system.jetcobot_command.serboway_control_server:main',
            'task_allocator = serboway_system.pinky_task_allocation.simplified_pinky_allocator:main',
        ],
    },
)