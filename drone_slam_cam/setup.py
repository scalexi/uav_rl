from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_slam_cam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'scipy', 'numpy'],
    zip_safe=True,
    maintainer='chahine',
    maintainer_email='mrchahine45@gmail.com',
    description='ROS2 package for 3D SLAM on UAVs using PX4 SITL, Gazebo, and RTAB-Map with depth camera.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_converter = drone_slam_cam.odom_converter:main'
        ],
    },
)
