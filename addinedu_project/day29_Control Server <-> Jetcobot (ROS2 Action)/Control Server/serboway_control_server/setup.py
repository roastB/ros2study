from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'serboway_control_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        # 패키지 인덱스 마커와 package.xml 파일을 명시적으로 설치
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 필요한 경우 launch 파일 추가 (launch 디렉토리가 있는 경우)
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addineud',
    maintainer_email='your-email@example.com',
    description='Jetcobot Control Server',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_client = serboway_control_server.order_client:main',
        ],
    },
)