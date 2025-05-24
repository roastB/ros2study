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
            'order_save = serboway_system.order_save:main',
            #'order_http = serboway_system.order_http:main',
            'order_process = serboway_system.order_process:main',
            'task_allocator = serboway_system.pinky_task_allocation.simplified_pinky_allocator:main',
        ],
    },
)