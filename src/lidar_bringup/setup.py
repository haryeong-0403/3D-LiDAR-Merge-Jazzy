from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 기본 설정
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch 파일 포함
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),

        # urdf 파일 포함
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haryeong',
    maintainer_email='haryeongkim0403@gmail.com',
    description='Dual LiDAR bringup package with URDF and robot_state_publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
