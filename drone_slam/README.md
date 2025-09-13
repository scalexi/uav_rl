# Drone SLAM

A ROS2 package for running Lidar-based SLAM (Simultaneous Localization and Mapping) on a PX4 drone simulated in Gazebo. This project bridges PX4's NED (North-East-Down) coordinate frame to ROS2's ENU (East-North-Up) frame, handles odometry conversion, and integrates SLAM Toolbox for mapping using Lidar data.

![Demonstration of Drone Altitude Control](https://github.com/eOvic/PX4-ROS2-SLAM-Control/blob/main/drone_slam/media/SLAM.gif)

### Important:
Copy the content of the world folder inside: PX4-Autopilot/Tools/simulation/gz/worlds


The package includes:
- A launch file (`slam.launch.py`) to start the simulation, bridges, SLAM node, and visualization in RViz2.
- An odometry converter script (`odom_converter.py`) to transform PX4 vehicle odometry to ROS2-compatible format.
- Supporting setup files for building the package.
- Configuration files (e.g., `slam_params.yaml`) for SLAM Toolbox.
- World files (e.g., SDF models for Gazebo simulations with walls and a drone equipped with a 2D Lidar).
- A TD3 Algorithm to control the drone.

**Note:** This project uses a Python-based odometry converter by default (via the launch file), but a C++ version (`odom_converter.cpp`) is also provided for reference or alternative use.

## Prerequisites

- **Operating System:** Ubuntu 24.04 LTS.
- **ROS2 Version:** Jazzy Jalisco.
- **PX4 Autopilot:** Latest stable version (v1.14 or higher recommended). Set up PX4 SITL (Software-In-The-Loop) with Gazebo support.
- **Gazebo Version:** Harmonic 8.9.0 (ensure it's installed via ROS2 integration, e.g., `ros-jazzy-ros-gz`).
- Python 3.12+ (comes with Ubuntu 24.04).

Additional tools:
- `colcon` for building ROS2 workspaces.
- `rosdep` for installing system dependencies.

## Installation

1. **Install ROS2 Jazzy:**
   Follow the official [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation.html) for Ubuntu 24.04.
   ```
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-jazzy-desktop
   source /opt/ros/jazzy/setup.bash
   ```

2. **Install Gazebo Harmonic and ROS-GZ Packages:**
   ```
   sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-gz-harmonic
   ```

3. **Set up PX4:**
   - Install dependencies:
     ```
     sudo apt install git cmake make g++ ninja-build python3-pip
     pip install --user -U empy pyros-genmsg setuptools tomli jinja2 packaging jsonschema numpy psutil kconfiglib
     ```
   - Clone PX4 repository:
     ```
     git clone https://github.com/PX4/PX4-Autopilot.git --recursive
     cd PX4-Autopilot
     ```
   - Build PX4 SITL for Gazebo:
     ```
     make px4_sitl gz_x500
     ```

### Setup Micro XRCE-DDS Agent & Client
```commandline
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Build ROS 2 Workspace
We will need px4_msgs and px4_ros_com:

```commandline
mkdir -p ~/ws_ros2/src/
cd ~/ws_ros2/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/eOvic/ROS2-2DLidar-Gazebo-SLAM
cd ..
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```
### Install MAVSDK (installation must be in Conda env)
```commandline
pip install mavsdk
pip install aioconsole
pip install pygame
pip install numpy
```
Navigate to the package folder and run this command : 
```
pip install -r requirements.txt
```
### Additional Configs
- Put below lines in your bashrc:
```commandline
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=~/.gz/models
```

6. **Build the workspace:**
   ```
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage
### Terminal 1: Launch the PX4-Gazebo Simulation
### IMPORTANT: Before running the simulation copy the content of the world folder to the simulation gz folder in PX4

First, launch the PX4 SITL simulation. This command will start the PX4 flight controller and open a Gazebo window with the drone model.

```bash
# Navigate to your PX4 directory
cd ~/PX4-Autopilot/

# You must choose a world that has a ground plane.
PX4_GZ_WORLD=walls make px4_sitl gz_x500_lidar_2d
```

Wait for the simulation to load completely and for the PX4 terminal to be ready for takeoff.
### Terminal 2: Micro DDS
To ensure the connection between Gazebo, ROS and PX4 we use MicroXRCEAgent 
Open another Terminal and write this command : 
```
 MicroXRCEAgent udp4 -p 8888 
``` 
IMPORTANT NOTE : You must run QGroundControl in the backgroud :
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
### Terminal 3: SLAM

1. **Source the workspace:**
   ```
   source ~/ws_ros2/install/setup.bash
   ```

2. **Launch the SLAM setup:**
   ```
   ros2 launch drone_slam slam.launch.py
   ```

   This will:
   - Set simulation time.
   - Bridge Gazebo clock and Lidar data to ROS2 topics.
   - Run the odometry converter to publish `/odom` in ENU frame.
   - Publish a static TF transform between `base_link` and `link`.
   - Start SLAM Toolbox with parameters from `config/slam_params.yaml`.
   - Configure and activate the SLAM node.
   - Launch RViz2 for visualization.

3. **Monitor Topics:**
   - Odometry: `ros2 topic echo /odom`
   - Laser Scan: `ros2 topic echo /scan`
   - Map: `ros2 topic echo /map` (once SLAM is active).

4. **Customization:**
   - Edit `config/slam_params.yaml` for SLAM Toolbox tuning (e.g., odometry noise, scan matching parameters).
   - Modify the world SDF files in `worlds/` for different simulation environments.
   - If using the C++ odometry converter, compile it separately and update the launch file to use an executable node instead of the Python script.
### Terminal 4: Run the Control Via keyboard

Once the simulation is running, open a new terminal, source your ROS 2 workspace, and run the training script.

```bash
conda activate venv

# Source your workspace
source ~/ws_ros2/install/setup.bash

# Run the training script
python3 ~/ws_ros2/src/drone_slam/nodes/keyboard-mavsdk-test.py
```

## Troubleshooting

- **Coordinate Frame Issues:** Ensure PX4 is publishing in NED/FRD and the converter is handling transformations correctly. Check TF tree with `ros2 run tf2_tools view_frames`.
- **Gazebo Bridge Errors:** Verify ROS-GZ bridge packages are installed and the Gazebo world matches the model paths in the launch file (e.g., `/world/walls/model/x500_lidar_2d_0/...`).
- **Missing Dependencies:** Run `rosdep install` again if nodes fail to start.
- **Simulation Not Starting:** Launch PX4 SITL separately first to ensure Gazebo is running, then run the ROS2 launch file.

## License

Apache-2.0 (see `package.xml` for details).

## Contributing

Pull requests welcome! For major changes, open an issue first.


