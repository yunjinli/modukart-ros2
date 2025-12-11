import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    
    # --- CONFIGURATION ---
    # Update this path to where you saved your URDF
    share = get_package_share_directory('modukart')
    urdf_path = os.path.join(share, 'urdf', 'zukimo_mobile_fixed.urdf')

    # Read the URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true', # <--- Default to true for bag playback
            description='Use simulation (Gazebo/Bag) clock if true'),
        # 1. Robot State Publisher 
        # (Broadcasts the transforms from your URDF: base_link -> frame -> camera etc.)
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

        # 2. Joint State Publisher 
        # (Needed because you have non-fixed joints like wheels. 
        # This publishes "0" for them so the tree doesn't break.)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': False}] 
        ),
        
        
        # 3. RTAB-Map SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
            ]),
            launch_arguments={
                'rtabmap_args': '--delete_db_on_start',
                'rgb_topic': '/rgb/image_rect_color',
                'depth_topic': '/rgb/image_rect_depth',
                'camera_info_topic': '/camera/realsense/color/camera_info_over',
                
                # CRITICAL: This ties the SLAM to your URDF's base
                'frame_id': 'base_link', 
                # 'frame_id': 'camera-module', 
                # 'frame_id': 'camera_color_optical_frame', 
                'use_sim_time': 'true',
                # CRITICAL: This tells SLAM which URDF link is the camera
                # Use the name of the link we added in Step 3
                # 'odom_frame_id': 'odom_slam', # RTAB-Map creates this frame
                'approx_sync': LaunchConfiguration('use_sim_time'),
                
                'qos': '1',
                'rviz': 'true'
            }.items(),
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            output='screen',
            arguments=['0', '0.15', '0', '0', '0', '0', 'camera-module', 'camera_color_optical_frame']
        ),
    ])