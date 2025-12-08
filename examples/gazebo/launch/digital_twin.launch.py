#!/usr/bin/env python3
"""
Launch file for Digital Twin simulation.

This launch file starts:
1. Gazebo Harmonic with a specified world file
2. ros_gz_bridge for ROS 2 communication
3. Optional: robot_state_publisher for TF

Usage:
    ros2 launch digital_twin_examples digital_twin.launch.py world:=humanoid-world.sdf
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_digital_twin = get_package_share_directory('digital_twin_examples')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='minimal-world.sdf',
        description='Name of the world file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value=os.path.join(pkg_digital_twin, 'config', 'ros_gz_bridge.yaml'),
        description='Path to ros_gz_bridge configuration file'
    )

    # World file path
    # Note: In a real setup, worlds would be in a specific location
    world_file = PathJoinSubstitution([
        pkg_digital_twin, '..', '..', '..', 'examples', 'gazebo', 'worlds',
        LaunchConfiguration('world')
    ])

    # Gazebo Harmonic simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f'config_file:={LaunchConfiguration("bridge_config")}'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        bridge_config_arg,
        gz_sim,
        bridge,
    ])
