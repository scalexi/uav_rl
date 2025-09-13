# Drone SLAM Cam

ROS2 package for 3D SLAM on UAVs using PX4 SITL, Gazebo, and RTAB-Map with a depth camera. This project integrates a depth camera on a drone model (e.g., x500_depth_0 in Gazebo) for real-time 3D mapping and localization. It includes odometry conversion from PX4's NED/FRD frame to ROS's ENU/FLU frame, ROS-Gazebo bridges, depth image processing, and RTAB-Map for SLAM.

![Demonstration of Drone Altitude Control](https://github.com/eOvic/PX4-ROS2-SLAM-Control/blob/main/drone_slam_cam/media/obstacles.png)

Key components:
- **Odometry Conversion**: Transforms PX4 vehicle odometry to ROS-compatible format.
- **Depth Registration**: Aligns depth images to RGB using `depth_image_proc`.
- **SLAM**: Uses RTAB-Map with RGB-D data, point clouds, and IMU.
- **Visualization**: RViz configuration for 3D mapping.
- **Simulation**: Tested with PX4 SITL and Gazebo Harmonic.

This will:
- Bridge Gazebo topics to ROS (e.g., images, depth, IMU, clock).
- Convert PX4 odometry to ROS `/odom`.
- Publish static TF for camera link.
- Run depth registration.
- Start RTAB-Map for 3D SLAM.
- Launch RViz for visualization.

## Prerequisites

- **Operating System**: Ubuntu 24.04 LTS (recommended for ROS2 Jazzy compatibility).
- **ROS2 Version**: Jazzy Jalisco.
- **PX4 Autopilot**: v1.15 or later (tested with SITL).
- **Gazebo**: Harmonic (version 8.9.0).
- Python 3.12+ (comes with Ubuntu 24.04).

## Installation

### 1. Install ROS2 Jazzy
Follow the official ROS2 installation guide:
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
```
Add `source /opt/ros/jazzy/setup.bash` to your `~/.bashrc` for persistent setup.

### 2. Install PX4 Autopilot
Clone and set up PX4 for SITL with Gazebo:
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500  # Build for Gazebo (Harmonic is compatible via ROS-GZ bridge)
```
Note: Ensure Gazebo Harmonic is installed (see below). PX4 SITL uses Gazebo for simulation.

### 3. Install Gazebo Harmonic
Install via the official Ignition/Gazebo packages:
```bash
sudo apt-get update
sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-gz8  # Installs Gazebo Harmonic 8.9.0
```
Verify: `gz sim --version` should show Harmonic.

### 4. Set Up ROS Workspace
Create a ROS2 workspace and clone this repository:
```bash
mkdir -p ~/ws_ros2/src
cd ~/ws_ros2/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/eOvic/ROS2-Gazebo-RGBD-3D-SLAM.git  
```

### 5. Install Dependencies
- ROS dependencies (from `package.xml`):
  ```bash
  cd ~/ws_ros2
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
  ```
  This installs packages like `rclcpp`, `px4_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `sensor_msgs`, `rtabmap_slam`, `ros_gz_bridge`, etc.

- Python dependencies (via pip):
  ```bash
  pip install -r ~/ws_ros2/src/drone_slam_cam/requirements.txt
  ```

### 6. Build the Package
```bash
cd ~/ws_ros2
colcon build --packages-select drone_slam_cam
source install/setup.bash
```
Add `source ~/ws_ros2/install/setup.bash` to your `~/.bashrc`.

## Configuration
- **RTAB-Map Parameters**: Edit `~/ws_ros2/src/drone_slam_cam/config/rtabmap_params.yaml` for SLAM tuning (e.g., voxel size, ICP settings).
- Ensure your Gazebo world includes a drone model like `x500_depth_0` with a depth camera (IMX214 sensor) and IMU.

## Usage

### Run PX4 SITL with Gazebo
Start the simulation (adjust model/world as needed):
```bash
cd PX4-Autopilot
make px4_sitl gz_x500_depth  # Or your custom world with walls/drone model
```
### Micro DDS
To ensure the connection between Gazebo, ROS and PX4 we use MicroXRCEAgent 
Open another Terminal and write this command : 
```
 MicroXRCEAgent udp4 -p 8888 
``` 

IMPORTANT NOTE : You must run QGroundControl in the backgroud :
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

### Launch the SLAM Package
In a new terminal:
```bash
source ~/ws_ros2/install/setup.bash
ros2 launch drone_slam_cam slam_3d.launch.py
```

Fly the drone in QGroundControl or via PX4 commands to generate maps.
### Run the Control Via keyboard

Once the simulation is running, open a new terminal, source your ROS 2 workspace, and run the training script.

```bash
conda activate venv

# Source your workspace
source ~/ws_ros2/install/setup.bash

# Run the training script
python3 ~/ws_ros2/src/drone_slam/nodes/keyboard-mavsdk-test.py
```

## Troubleshooting
- **No Data in RViz**: Ensure PX4 SITL is running and topics are bridged correctly (check with `ros2 topic list`).
- **Frame Mismatches**: Verify NED-to-ENU transformations in `odom_converter.py`.
- **Gazebo Version Issues**: If using older Gazebo, switch to `gazebo-classic` in PX4 build.
- **Performance**: Increase queue sizes in depth proc if sync issues occur.

## License
Apache-2.0 (as per `setup.py`).


