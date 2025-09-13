# File: px4_rl_project/launch/x500_nav_ppo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="px4_rl_project",
            executable="train_agent",
            name="train_agent_eval",
            output="screen",
            arguments=["--eval"]
        ),
    ])
