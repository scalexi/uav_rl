# File: px4_rl_project/launch/x500_all_in_one.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    world = LaunchConfiguration("world")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="/home/$USER/ws_ros2/src/px4_rl_project/worlds/x500_depth.sdf"
        ),

        # Start Gazebo Harmonic with your world
        Node(
            package="ros_gz_sim",
            executable="gz_sim",
            name="gz_sim",
            output="screen",
            arguments=["-v", "4", "-r", world]
        ),

        # Bridge RGB image
        Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="rgb_bridge",
            arguments=["/camera"],
            output="screen"
        ),

        # Bridge Depth image
        Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="depth_bridge",
            arguments=["/camera/depth_image"],
            output="screen"
        ),

        # YOLO bringup
        Node(
            package="yolo_bringup",
            executable="yolo",
            name="yolo_detector",
            parameters=[{
                "input_image_topic": "/camera/image",
                "input_depth_topic": "/camera/depth_image",
                "input_depth_info_topic": "/camera/camera_info",
                "model": "yolov8m.pt",
                "use_3d": False
            }],
            output="screen"
        ),

        # Our fusion node -> publishes /obstacle_info
        Node(
            package="px4_rl_project",
            executable="yolo_depth_fusion",
            name="yolo_depth_fusion",
            output="screen"
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
            # arguments=["-d", "/home/$USER/ws_ros2/src/px4_rl_project/rviz/x500_yolo.rviz"]
        ),
    ])
