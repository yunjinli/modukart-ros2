import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Path to your working "Blue Box" URDF
    share = get_package_share_directory('modukart')
    urdf_file = os.path.join(share, 'urdf', 'simple_box.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        
        # --- ROBOT STATE ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf_file]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

        # --- SLAM (RTAB-Map) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
            ]),
            launch_arguments={
                'rtabmap_args': '--delete_db_on_start',
                'rgb_topic': '/rgb/image_rect_color',
                'depth_topic': '/rgb/image_rect_depth',
                'camera_info_topic': '/camera/realsense/color/camera_info_over',
                'frame_id': 'base_link',
                'approx_sync': 'true',
                'qos': '1',
                'rviz': 'true'
            }.items(),
        ),

        # --- TF BRIDGE 1: Robot to Camera Data (The fix you found!) ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_bridge',
            # Connects URDF 'camera-module' to Data 'camera_color_optical_frame'
            arguments=['0', '0', '0', '0', '0', '0', 'camera-module', 'camera_color_optical_frame']
        ),

        # --- TF BRIDGE 2: Robot to Downward Sensor (For Semantics) ---
        # Positioning it 20cm forward, pointing DOWN (pitch 1.57)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='down_sensor_bridge',
            arguments=['0.2', '0', '0', '0', '1.57', '0', 'base_link', 'down_sensor_link']
        ),
    ])