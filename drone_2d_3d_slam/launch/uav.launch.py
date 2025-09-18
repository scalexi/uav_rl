#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, SetParameter, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from math import radians

def generate_launch_description():
    ld = LaunchDescription()

    ns = 'uav'

    # Add SLAM type selection argument
    slam_type_arg = DeclareLaunchArgument(
        'slam_type',
        default_value='2d',
        description='Type of SLAM to use: 2d (slam) or 3d (slam)',
        choices=['2d', '3d']
    )
    
    slam_type = LaunchConfiguration('slam_type')

    # Node for Drone 1
    world = {'gz_world': 'default'}
    model_name = {'gz_model_name': 'x500_lidar_rgbd'}
    autostart_id = {'px4_autostart_id': '4020'}
    # autostart_id = {'px4_autostart_id': '4020'}
    instance_id = {'instance_id': '1'}
    xpos = {'xpos': '0.0'}
    ypos = {'ypos': '0.0'}
    zpos = {'zpos': '0.0'}
    headless = {'headless': '0'}

    # PX4 SITL + Spawn x500_d435
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_2d_3d_slam'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_ns': ns,
            'headless': headless['headless'],
            'gz_world': world['gz_world'],
            'gz_model_name': model_name['gz_model_name'],
            'px4_autostart_id': autostart_id['px4_autostart_id'],
            'instance_id': instance_id['instance_id'],
            'xpos': xpos['xpos'],
            'ypos': ypos['ypos'],
            'zpos': zpos['zpos']
        }.items()
    )

    # MAVROS
    package_share_directory = get_package_share_directory('drone_2d_3d_slam')
    plugins_file_path = os.path.join(package_share_directory, 'config', 'uav_px4_pluginlists.yaml')
    config_file_path = os.path.join(package_share_directory, 'config', 'uav_px4_config.yaml')
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_2d_3d_slam'),
                'launch',
                'mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'mavros_namespace': ns + '/mavros',
            'tgt_system': '2',
            'fcu_url': 'udp://:14541@127.0.0.1:14558',
            'pluginlists_yaml': plugins_file_path,
            'config_yaml': config_file_path,
            'base_link_frame': 'uav/base_link',
            'odom_frame': 'uav/odom',
            'map_frame': 'map',
            'use_sim_time': 'True'
        }.items()
    )

    odom_frame = 'odom'
    base_link_frame = 'base_link'

    # Static TF map/world -> local_pose_ENU
    map_frame = 'map'
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_' + ns + '_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), str(zpos['zpos']), '0.0', '0', '0', map_frame, ns + '/' + odom_frame],
    )

    # Static TF base_link -> depth_camera
    cam_x = 0.12
    cam_y = 0.03
    cam_z = 0.242
    cam_roll = 0.0
    cam_pitch = 0.0
    cam_yaw = 0.0
    cam_tf_node = Node(
        package='tf2_ros',
        name=ns + '_base2depth_tf_node',
        executable='static_transform_publisher',
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns + '/' + base_link_frame, 'camera_link'],
    )
    # Static TF base_link -> lidar
    lidar_x = -0.1
    lidar_y = 0.0
    lidar_z = 0.26
    lidar_roll = 0.0
    lidar_pitch = 0.0
    lidar_yaw = 0.0
    lidar_tf_node = Node(
        package='tf2_ros',
        name=ns + '_base2lidar_tf_node',
        executable='static_transform_publisher',
        arguments=[str(lidar_x), str(lidar_y), str(lidar_z), str(lidar_roll), str(lidar_pitch), str(lidar_yaw), ns + '/' + base_link_frame, 'laser_link'],
    )

    # Transport rgb and depth images from GZ topics to ROS topics    
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        name='ros_bridge_node',
        executable='parameter_bridge',
        arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/rgb_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/depth_image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/world/default/model/x500_lidar_rgbd_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        

        '--ros-args',
        '-r', '/rgb_image:=' + ns + '/rgb/image_rect',
        '-r', '/rgb/camera_info:=' + ns + '/rgb/camera_info',
        '-r', '/depth_image:=' + ns + '/depth/image_rect',
        '-r', '/depth/camera_info:=' + ns + '/depth/camera_info',
        '-r', '/depth_image/points:=' + ns + '/depth/points',
        '-r', '/scan:=' + ns + '/scan',
        '-r', '/world/default/model/x500_lidar_rgbd_0/link/base_link/sensor/imu_sensor/imu:=' + ns + '/imu',
        ]

    )
    # SLAM Toolbox for 2D SLAM
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(package_share_directory, 'config', 'slam_params.yaml')],
        output='screen',
        # condition=IfCondition(lambda context: context.launch_configurations['slam_type'] == '2d')
    )

    # Configure SLAM Toolbox
    configure_slam_toolbox = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen',
        # condition=IfCondition(lambda context: context.launch_configurations['slam_type'] == '2d')
    )

    # Activate SLAM Toolbox
    activate_slam_toolbox = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen',
        # condition=IfCondition(lambda context: context.launch_configurations['slam_type'] == '2d')
    )

    # Depth processing container for 3D SLAM
    depth_proc_container = ComposableNodeContainer(
        name='depth_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::RegisterNode',
                name='register_node',
                parameters=[{
                    'queue_size': 20,
                    'exact_sync': False,
                }],
                # depth_image_proc::RegisterNode
                remappings=[
                ('rgb/image_rect',        ns + '/rgb/image_rect'),
                ('rgb/camera_info',       ns + '/rgb/camera_info'),
                ('depth/image_rect',      ns + '/depth/image_rect'),
                ('depth/camera_info',     ns + '/depth/camera_info'),
                ('depth_registered/image_rect', ns + '/depth_registered/image'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        # condition=IfCondition(lambda context: context.launch_configurations['slam_type'] == '3d')
    )
    # RTAB-Map for 3D SLAM (RGB-D + optional LiDAR/PointCloud + IMU)
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[os.path.join(package_share_directory, 'config', 'rtabmap_params.yaml'),
                    {'use_sim_time': True},
        ],
        arguments=['-d'],
        remappings=[
            # RGB-D inputs
            ('rgb/image',         ns + '/rgb/image_rect'),
            ('rgb/camera_info',   ns + '/rgb/camera_info'),
            ('depth/image',       ns + '/depth_registered/image'),
            ('depth/camera_info', ns + '/depth/camera_info'),

            # Use ONE of these (or both if your params enable them):
            #   - 'scan' for 2D LiDAR
            #   - 'scan_cloud' for 3D point cloud
            # ('scan',              ns + '/scan'),
            ('scan_cloud',        ns + '/depth/points'),

            # Optional but recommended if available
            ('imu',               ns + '/imu'),
            ('odom',              ns + '/odom'),
        ],
        output='screen',
        # condition=IfCondition(lambda context: context.launch_configurations['slam_type'] == '3d')
    )

    # ========================
    # SLAM INTEGRATION SECTION
    # ========================

    # Conditional RViz configurations
    # RViz2 for 2D SLAM
    rviz_node_2d = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_2d',
        output='screen',
        # condition=IfCondition(lambda context: context.launch_configurations['slam_type'] == '2d')
    )

    # Add all actions to launch description
    ld.add_action(slam_type_arg)
    ld.add_action(gz_launch)
    ld.add_action(map2pose_tf_node)
    ld.add_action(cam_tf_node)
    ld.add_action(lidar_tf_node)
    ld.add_action(ros_gz_bridge)
    ld.add_action(mavros_launch)
    
    # Add 2D SLAM nodes
    ld.add_action(slam_toolbox_node)
    ld.add_action(configure_slam_toolbox)
    ld.add_action(activate_slam_toolbox)
    ld.add_action(rviz_node_2d)
    
    # Add 3D SLAM nodes
    # ld.add_action(depth_proc_container)
    # ld.add_action(rtabmap_node)
    # ld.add_action(rviz_node_3d)

    return ld