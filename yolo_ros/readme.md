If you want to bridge the camera topic of the drones and apply yolo obstacle detection use these commands :

## Commands
```bash
git clone https://github.com/mgonzs13/yolo_ros.git
pip3 install -r yolo_ros/requirements.txt
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
```bash

ros2 run ros_gz_image image_bridge /camera/rgb/image_raw /camera/rgb/camera_info /camera/depth/image_raw:=/depth_image /camera/depth/camera_info
```

```bash
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/camera input_depth_topic:=/depth_image input_depth_info_topic:=/camera/depth/camera_info model:=yolov11m.pt use_3d:=True
```

```bash
ros2 run ros_gz_image image_bridge /camera /depth_camera --ros-args --remap /depth_camera:=/depth_image
```

```bash
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image model:=yolov8m.pt use_3d:=True input_depth_topic:=/depth_camera input_depth_info_topic:=/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info
```

