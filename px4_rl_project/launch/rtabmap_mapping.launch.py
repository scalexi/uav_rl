from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Optimizer/Iterations': '20',
                'RGBD/AngularUpdate': '0.1',
                'RGBD/LinearUpdate': '0.1',
                'Mem/RehearsalSimilarity': '0.45',
                'Grid/3D': 'true',
                'Grid/RangeMax': '20.0',
                'Grid/PreVoxelFiltering': 'true',
                'Grid/RayTracing': 'true',
            }],
            remappings=[
                ('rgb/image', '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                ('depth/image', '/depth_camera'),
                ('odom', '/fmu/out/vehicle_odometry')
            ]
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_topic': '/fmu/out/vehicle_odometry',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image', '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                ('rgb/camera_info', '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                ('depth/image', '/depth_camera'),
                ('odom', '/fmu/out/vehicle_odometry')
            ]
        )
    ])
