# parent.launch.py
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution as P
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    teleop_pkg = FindPackageShare('teleop_control')
    sensor_pkg = FindPackageShare('sensor_pipeline')

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(P([teleop_pkg, 'launch', 'joystick_teleop.launch.py'])),
        launch_arguments={'device_port': '/dev/xmc1100'}.items()
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(P([sensor_pkg, 'launch', 'launch_sensors.launch.py'])),
        # launch_arguments={'device_port': '/dev/xmc1100'}.items()
    )

    return LaunchDescription([teleop, sensors])
