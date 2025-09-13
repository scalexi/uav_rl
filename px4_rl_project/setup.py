import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'px4_rl_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Finds 'environment' and other sub-packages automatically
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'cv_bridge',
        'yolo_msgs'
    ],
    zip_safe=True,
    maintainer='chahine',
    maintainer_email='chahine@chahine-Pc',
    description='DRL for PX4 Drone Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_agent = px4_rl_project.train_agent:main',
            'train_nav_agent = px4_rl_project.train_nav_agent:main',
            'odom_to_tf = nodes.odom_to_tf:main',
            'obstacle_distance = nodes.obstacle_distance:main',
        ],
    },
)

