import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import math
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution as P

def generate_launch_description():
    share = get_package_share_directory('modukart')
    urdf_path = os.path.join(share, 'urdf', 'zukimo_mobile_fixed.urdf')
    
    # Read the URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    sensor_pkg = FindPackageShare('sensor_pipeline')

    depth_overlay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(P([sensor_pkg, 'launch', 'depth_overlay_headless.launch.py'])),
    )
    
    radar_pkg = FindPackageShare('bgt60tr13c_driver')
    radar_surface_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(P([radar_pkg, 'launch', 'radar_surface_detection.launch.py'])),
    )
    
    radar_semantic_mapping_node = Node(
            package='sensor_pipeline',
            executable='radar_semantic_mapping',
            name='radar_semantic_surface_mapping',
            output='screen',
            parameters=[],
        )
    
    return LaunchDescription([
        radar_surface_detection,
        depth_overlay,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo/Bag) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': LaunchConfiguration('use_sim_time')
                         }],
            arguments=[urdf_path]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': False}] 
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
            ]),
            launch_arguments={
                'rtabmap_args': (
                    '--delete_db_on_start '              
                    '--Cloud/VoxelSize 0.01 '
                    '--Vis/MaxFeatures 2000 '
                    '--Reg/Force3DoF true '
                    '--Grid/DepthDecimation 1 '
                    '--Grid/3D true '              
                    '--Grid/FromDepth true '
                    # '--Rtabmap/DetectionRate 3 '
                    # '--RGBD/LinearUpdate 0.05 '      
                    # '--RGBD/AngularUpdate 0.05 '
                    '--Rtabmap/DetectionRate 0 '       
                    '--RGBD/LinearUpdate 0.1 '         
                    '--RGBD/AngularUpdate 0.1 '        
                    '--Vis/MinInliers 15 '             
                ),
                'rgb_topic': '/rgb/image_rect_color',
                'depth_topic': '/rgb/image_rect_depth',
                'camera_info_topic': '/camera/realsense/color/camera_info_over',
                'frame_id': 'base_link', 
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'approx_sync': 'true',
                'qos': '1', 
                'rviz': 'true',
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            output='screen',
            arguments=['0', '0.2', '0', '0', '0', '0', 'camera-module', 'camera_color_optical_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='radar_link_broadcaster',
            output='screen',
            arguments=['0', '0.475', '0.1', '0', '0', f'-{str(math.pi / 2)}', 'camera-module', 'radar_link']
        ),
        radar_semantic_mapping_node,
    ])