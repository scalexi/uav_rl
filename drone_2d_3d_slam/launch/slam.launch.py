import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1) Global sim time
        SetParameter(name='use_sim_time', value=True),

        # 2) Bridge Gazebo /clock -> ROS
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        # 3) Lidar bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            ros_arguments=['-r', '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/scan'],
            output='screen'
        ),

        # 4) Odom converter
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/ros2_ws/src/uav_rl/drone_2d_3d_slam/drone_2d_3d_slam/odom_converter.py')],
            output='screen'
        ),

        # 5) Static TF base_link -> link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.26', '0', '0', '0', 'base_link', 'link'],
            output='screen'
        ),

        # 6) SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.expanduser('~/ros2_ws/src/uav_rl/drone_2d_3d_slam/config/slam_params.yaml')],
            output='screen'
        ),

        # 7) Configure SLAM Toolbox
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
            output='screen'
        ),

        # 8) Activate SLAM Toolbox
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
            output='screen'
        ),

        # 9) RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
