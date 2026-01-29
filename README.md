# Modukart Platform

```
cd <path/to/ros2_ws>/src
git clone --recursive https://github.com/yunjinli/modukart-ros2.git
cd ../
## Make sure you download the flexx2 sdk already
colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=<royale_sdk_path>/lib/cmake/
```

## Introduction
This is the modular kart platform based on ROS2. The project consists of three pipelines:
- `control_pipeline`: Sending actuation command to drive the motors.
- `sensor_pipeline`: ROS2 packages for launching realsense, ToF, USB camera, bgt60tr13c, IMU. Launching multiple sensors for sensor fusion application, e.g. colorized ToF pointcloud with RGB, and depth image overlay.
- `data_pipeline`: For recording sync image pairs for intrinsic/extrinsic calibration. 

## Bring-Up (Launching motors and sensors)

- `sensor_pipeline`: Launch radar, tof, rgb, imu
- `control_pipeline`: Launch teleop gamepad on motor
- For recording data, simply run this and then do `ros2 bag record <topic_name> ## Or simply pass --all`

```bash
## vis: Visualize radar data using matplotlib
## run_rviz: Visualize data streaming from rviz
ros2 launch modukart bringup.launch.py vis:=true run_rviz:=true 
```

## URDF + depth_overlay + rtabmap

### Pre-requisite
- intrinsics/extrinsic calibration (if the camera are not changed and the camera holder doesn't change, you can use the default setting)

### Run

Play the data:

```bash
ros2 bag play rosbag2_2025_11_13-15_14_31 --topics /pmd_royale_ros_camera_node/camera_info /pmd_royale_ros_camera_node/depth_image_0 /pmd_royale_ros_camera_node/gray_image_0 /pmd_royale_ros_camera_node/point_cloud_0 /camera/realsense/color/camera_info /camera/realsense/color/image_raw --clock
```

Depth overlay:

```bash
ros2 launch sensor_pipeline depth_overlay_headless.launch.py
```

Run rtabmap

```bash
ros2 launch modukart modukart_slam.launch.py
```

In the pop-up Rviz, add `RobotModel`, and select `Description Topic` to `/robot_description`

## Ground Semantic Mapping Demo

### Overview
1. We assume that the intrinsic/extrinsic calibration are known, we apply these parameters to overlay the depth value on rgb image.
2. Then we run rtabmap with rgb-d camera input to simultaneously localize the camera and build the 3D map (colorized pointcloud)
3. We run the radar surface semantic mapping, which takes a pretrained radar surface detection model. We treat the radar as a downward-facing camera frustum to render the point inside its fov to the corresponding colors representing different classes.

### Pre-requisite
- intrinsics/extrinsic calibration (if the camera are not changed and the camera holder doesn't change, you can use the default setting)
- Need a pretrained radar surface classifier based on bgt60tr13c

### Demo

https://github.com/user-attachments/assets/11825dd5-d864-4f83-88e0-66c57aa02558

### Run

Play the data:

```bash
ros2 bag play rosbag2_2025_11_13-15_14_31 --topics /pmd_royale_ros_camera_node/camera_info /pmd_royale_ros_camera_node/depth_image_0 /pmd_royale_ros_camera_node/gray_image_0 /pmd_royale_ros_camera_node/point_cloud_0 /camera/realsense/color/camera_info /camera/realsense/color/image_raw /ifx_bgt_device/bgt60tr13c/device_config /ifx_bgt_device/bgt60tr13c/raw_frame --clock
```

```bash
ros2 launch modukart modukart_ground_semantic_mapping.launch.py model_type:=onnx
```
