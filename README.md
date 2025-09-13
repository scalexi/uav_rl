# Drone SLAM + YOLO integration 

![Demonstration of Drone Altitude Control](https://github.com/eOvic/PX4-ROS2-SLAM-Control/blob/main/drone_slam/media/ROS.png)

This repo contains files that every robotics UAV engineer would need :

This repository integrates **PX4**, **ROS 2 Jazzy**, and **Gazebo Harmonic** for drone navigation and control using **SLAM**.  
It provides launch files, configuration, and example workflows to visualize LiDAR scans, build maps, and control a drone in simulation.

## Features:

- drone_slam: start the simulation, bridges, SLAM node, and visualization in RViz2 using 2D lidar .
- drone_slam_cam: Uses a RGBD camera mounted on the drone for SLAM and point cloud extraction.
- px4_rl_project: this folder contains a DQN approach for drone hovering and altitude maintenance

- yolo_ros repo could be exploited in future usage to do object detection and S&R tasks-- commands to use yolo_ros will be found in the folder. 
- An odometry converter script (`odom_converter.py`) to transform PX4 vehicle odometry to ROS2-compatible format (needed in Rviz to establish a proper TF tree).
- Keyboard control node which you could find in drone_slam and drone_slam_cam

**Note:** This project uses a Python-based odometry converter by default (via the launch file), but a C++ version (`odom_converter.cpp`) is also provided for alternative use.
**Note:** This repo is for having a good setup in place, future control integration and Nav2 or SkyPilot deployment will follow later !!

## Needed Repos
### 1. Install prerequisites

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions git
```

---
### 2. Create a workspace

```bash
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws/src
```

---
### 3. Clone the repos

```bash
# px4_msgs contains ROS2 message definitions
git clone https://github.com/PX4/px4_msgs.git

# px4_ros_com contains the microRTPS bridge & ROS2 interfaces
git clone https://github.com/PX4/px4_ros_com.git
```
### 4. Build the workspace

From the workspace root:

```bash
cd ~/px4_ros2_ws
colcon build
```

---
### 5. Source the workspace

```bash
source install/setup.bash
```

If you want it permanent:

```bash
echo "source ~/px4_ros2_ws/install/setup.bash" >> ~/.bashrc
```

---
### 6. Verify installation

Check if the PX4 messages are available:

```bash
ros2 interface show px4_msgs/msg/VehicleOdometry
```

If it prints the message definition → ✅ you installed `px4_msgs`.
Also check:

```bash
ros2 pkg list | grep px4
```


PX4-ROS2-SLAM-Control/
 ├── drone_slam/            # SLAM with Lidar 2D, TD3 control and keyboard control 
 ├── drone_slam_cam/        # SLAM with point cloud and RGBD
 ├── px4_rl_project/         # Reinfocement learning Control for drone hovering
 ├── yolo_ros/                  # Commands for yolo_ros that you will need
 └── README.md
## Known Issues

- Setting fixed frame to `map` requires SLAM to be active.
    
- `odom` frame may appear rotated vs `base_link`.
## Contributions 
⭐⭐ If you found these projects interesting don't forget that star ⭐⭐ !! 

