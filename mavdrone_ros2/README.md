# Mavdrone for ROS2
ROS2 packages to connect to a DJI drone using MSDK5 and Mavlink. 

## Installation
1. Download the packages from this folder and place them in the `src` folder of your workspace. 
2. Install `mavdrone_python`:
```bash
pip install "mavdrone @ git+https://github.com/WUR-ABE/mavdrone.git@main#subdirectory=mavdrone_python"
```
3. Install the ROS dependencies:
```bash
rosdep install --from-paths src --ignore-src -y 
```
4. Build your workspace:
```bash
colcon build
source install/setup.bash
```

## Packages

### m350_description
Contains a simplified URDF of a DJI M350 drone with a Zenmuse P1 camera with the correct gimbal joints.

### mavdrone_msgs
Contains the messages and actions to control the drone.

### mavdrone_ros
Contains the nodes to launch the mavdrone connection, a local grid broadcaster and a goal pose server to send RVIZ
navigation goals to mavdrone.

## Usage
1. Setup and configure [`mavlink-router`](https://github.com/WUR-ABE/mavdrone/tree/main?tab=readme-ov-file#configure-mavlink-router).
2. Verify the drone IP in `[TcpEndpoint Drone]` (`Address`) in [`mavdrone_python/router.conf`](../mavdrone_python/router.conf) matches the server address shown in the Android app.
3. Launch `mavdrone_ros` using:
```bash
ros2 launch mavdrone_ros drone.launch.py rviz:=true drone:=true
```
4. Select a 2D navigation goal in RVIZ. 
> **⚠️ Caution:** This will start the drone, takeoff and fly to the specified location (if within the distance limit of the drone). Test your code first using the DJI simulation mode.

## Calibrate drone camera
To calibrate the camera, run:
```bash
sudo apt-get install ros-$ROS_DISTRO-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/mavdrone/camera_image camera:=/mavdrone
```

Save the resulting camera matrix and distortion coefficients in [`mavdrone_ros/config/camera_info.yaml`](mavdrone_ros/config/camera_info.yaml).