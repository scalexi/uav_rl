import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Try to get PX4_DIR from environment variable first
    PX4_DIR = os.getenv('PX4_DIR')
    
    # If not set as environment variable, try common locations
    if PX4_DIR is None:
        # Try home directory first
        home_px4_path = os.path.expanduser('~/PX4-Autopilot')
        if os.path.exists(home_px4_path):
            PX4_DIR = home_px4_path
            print(f'Found PX4-Autopilot in home directory: {PX4_DIR}')
        else:
            # Try other common locations
            common_paths = [
                os.path.expanduser('~/px4'),
                os.path.expanduser('~/PX4'),
                os.path.expanduser('~/workspace/PX4-Autopilot'),
                '/opt/PX4-Autopilot'
            ]
            
            for path in common_paths:
                if os.path.exists(path):
                    PX4_DIR = path
                    print(f'Found PX4 installation at: {PX4_DIR}')
                    break
    
    if PX4_DIR is not None:
        print(f'Using PX4_DIR: {PX4_DIR}')
        
        # Verify that the px4 binary exists
        px4_binary = os.path.join(PX4_DIR, 'build/px4_sitl_default/bin/px4')
        if not os.path.exists(px4_binary):
            print(f'Warning: PX4 binary not found at {px4_binary}')
            print('Make sure PX4 is built with: make px4_sitl_default')
    else:
        print('PX4_DIR is not set and PX4-Autopilot not found in common locations')
        print('Please either:')
        print('1. Set PX4_DIR environment variable: export PX4_DIR=/path/to/PX4-Autopilot')
        print('2. Install PX4-Autopilot in ~/PX4-Autopilot')
        print('3. Create a symlink: ln -s /path/to/your/px4 ~/PX4-Autopilot')
        sys.exit(1)

    namespace = LaunchConfiguration('gz_ns')
    namespace_launch_arg = DeclareLaunchArgument(
        'gz_ns',
        default_value=''
    )

    headless = LaunchConfiguration('headless')
    headless_launch_arg = DeclareLaunchArgument(
        'headless',
        default_value='0'
    )

    gz_world = LaunchConfiguration('gz_world')
    gz_world_launch_arg = DeclareLaunchArgument(
        'gz_world',
        default_value='default'
    )

    gz_model_name = LaunchConfiguration('gz_model_name')
    gz_model_name_launch_arg = DeclareLaunchArgument(
        'gz_model_name',
        default_value='x500'
    )

    px4_autostart_id = LaunchConfiguration('px4_autostart_id')
    px4_autostart_id_launch_arg = DeclareLaunchArgument(
        'px4_autostart_id',
        default_value='4001'
    )

    instance_id = LaunchConfiguration('instance_id')
    instance_id_launch_arg = DeclareLaunchArgument(
        'instance_id',
        default_value='0'
    )

    xpos = LaunchConfiguration('xpos')
    xpos_launch_arg = DeclareLaunchArgument(
        'xpos',
        default_value='0.0'
    )

    ypos = LaunchConfiguration('ypos')
    ypos_launch_arg = DeclareLaunchArgument(
        'ypos',
        default_value='0.0'
    )

    zpos = LaunchConfiguration('zpos')
    zpos_launch_arg = DeclareLaunchArgument(
        'zpos',
        default_value='0.2'
    )

    # Create the PX4 SITL process
    px4_sim_process = ExecuteProcess(
        cmd=[[
            'cd ', PX4_DIR, ' && ',
            'PX4_SYS_AUTOSTART=', px4_autostart_id,
            ' PX4_GZ_MODEL=', gz_model_name,
            ' PX4_UXRCE_DDS_NS=', namespace,
            " PX4_GZ_MODEL_POSE='", xpos, ',', ypos, ',', zpos, "'",
            ' PX4_GZ_WORLD=', gz_world,
            ' ./build/px4_sitl_default/bin/px4 -i ', instance_id
        ]],
        shell=True,
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(headless_launch_arg)
    ld.add_action(gz_world_launch_arg)
    ld.add_action(gz_model_name_launch_arg)
    ld.add_action(px4_autostart_id_launch_arg)
    ld.add_action(instance_id_launch_arg)
    ld.add_action(xpos_launch_arg)
    ld.add_action(ypos_launch_arg)
    ld.add_action(zpos_launch_arg)
    ld.add_action(namespace_launch_arg)
    ld.add_action(px4_sim_process)

    return ld