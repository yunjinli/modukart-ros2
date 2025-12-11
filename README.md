# Modukart Platform

ROS 2 Project for Modular Kart Platform

## Bring-Up

```bash
ros2 launch modukart bringup.launch.py vis:=true run_rviz:=true
```

## URDF + depth_overlay + rtabmap

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
