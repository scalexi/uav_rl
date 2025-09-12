import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/world/walls/model/x500_depth_0/link/camera_link/sensor/camera_imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
            ],
            ros_arguments=[
                '-r', '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/depth_camera/image_raw',
                '-r', '/depth_camera:=/depth_camera/depth/image_rect_raw',
                '-r', '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info:=/depth_camera/camera_info',
                '-r', '/camera_info:=/depth_camera/depth/camera_info',
                '-r', '/depth_camera/points:=/depth_camera/points',
                '-r', '/world/walls/model/x500_depth_0/link/camera_link/sensor/camera_imu/imu:=/depth_camera/imu'
            ],
            output='screen'
        ),

        Node(
            package='drone_slam_cam',
            executable='odom_converter',
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.12', '0.03', '0.242', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        ),

        # Register depth to RGB frame (aligns and resizes depth to match RGB resolution/intrinsics)
        ComposableNodeContainer(
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
                        'queue_size': 20,  # Larger queue for better sync tolerance
                        'exact_sync': False,  # Use approximate sync
                    }],
                    remappings=[
                        ('rgb/image_rect', '/depth_camera/image_raw'),
                        ('rgb/camera_info', '/depth_camera/camera_info'),
                        ('depth/image_rect', '/depth_camera/depth/image_rect_raw'),
                        ('depth/camera_info', '/depth_camera/depth/camera_info'),
                        ('depth_registered/image_rect', '/depth_camera/depth/registered'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[os.path.expanduser('~/ws_ros2/src/drone_slam_cam/config/rtabmap_params.yaml')],
            arguments=['-d'],
            remappings=[
                ('rgb/image', '/depth_camera/image_raw'),
                ('depth/image', '/depth_camera/depth/registered'),  # Registered depth
                ('rgb/camera_info', '/depth_camera/camera_info'),  # RGB info (matches registered depth)
                ('scan_cloud', '/depth_camera/points'),
                ('imu', '/depth_camera/imu'),
                ('odom', '/odom')
            ],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.expanduser('~/ws_ros2/src/drone_slam_cam/config/rviz_3d.rviz')],
            output='screen'
        ),
    ])