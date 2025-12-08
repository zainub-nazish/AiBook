#!/usr/bin/env python3
"""
Isaac ROS Full Perception Pipeline Launch File.

This launch file demonstrates a complete perception pipeline:
- Stereo depth processing
- Visual SLAM for localization
- nvblox for 3D mapping

Usage:
    ros2 launch isaac_ros perception_pipeline.launch.py

Requirements:
    - Isaac ROS packages installed
    - Stereo camera or RGB-D camera
    - NVIDIA GPU with 8GB+ VRAM
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for full perception pipeline."""

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time from /clock'
    )

    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='stereo',
        description='Camera type: stereo or rgbd'
    )

    # Perception container with NITROS
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # =========================================
            # DEPTH PROCESSING
            # =========================================
            # Disparity computation (stereo → disparity)
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::DisparityNode',
                name='disparity_node',
                parameters=[{
                    'max_disparity': 64.0,
                    'backends': 'CUDA',
                }],
                remappings=[
                    ('left/image_rect', '/left/image_rect'),
                    ('left/camera_info', '/left/camera_info_rect'),
                    ('right/image_rect', '/right/image_rect'),
                    ('right/camera_info', '/right/camera_info_rect'),
                ]
            ),

            # Disparity to depth conversion
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
                name='disparity_to_depth',
                parameters=[],
            ),

            # Point cloud from depth
            ComposableNode(
                package='isaac_ros_depth_image_proc',
                plugin='nvidia::isaac_ros::depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_node',
                parameters=[{
                    'skip': 1,  # Process every frame
                }],
                remappings=[
                    ('depth/image_rect_raw', '/depth/image'),
                    ('depth/camera_info', '/depth/camera_info'),
                    ('rgb/image_rect_color', '/left/image_rect'),
                    ('rgb/camera_info', '/left/camera_info_rect'),
                ]
            ),

            # =========================================
            # VISUAL SLAM
            # =========================================
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'enable_rectified_pose': True,
                    'rectified_images': True,
                    'image_height': 480,
                    'image_width': 640,
                    'enable_slam_visualization': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                }],
                remappings=[
                    ('stereo_camera/left/image', '/left/image_rect'),
                    ('stereo_camera/left/camera_info', '/left/camera_info_rect'),
                    ('stereo_camera/right/image', '/right/image_rect'),
                    ('stereo_camera/right/camera_info', '/right/camera_info_rect'),
                ]
            ),

            # =========================================
            # NVBLOX 3D MAPPING
            # =========================================
            ComposableNode(
                package='nvblox_ros',
                plugin='nvblox::NvbloxNode',
                name='nvblox_node',
                parameters=[{
                    # Map parameters
                    'voxel_size': 0.05,  # 5cm voxels
                    'esdf': True,
                    'esdf_2d': True,

                    # Integration parameters
                    'max_integration_distance_m': 7.0,
                    'lidar_projective_integrator_max_integration_distance_m': 10.0,

                    # Output
                    'mesh': True,
                    'slice_height': 0.5,

                    # Frame IDs
                    'global_frame': 'map',

                    # Performance
                    'max_poll_rate_hz': 100.0,
                }],
                remappings=[
                    ('depth/image', '/depth/image'),
                    ('depth/camera_info', '/depth/camera_info'),
                    ('color/image', '/left/image_rect'),
                    ('color/camera_info', '/left/camera_info_rect'),
                ]
            ),
        ],
        output='screen',
    )

    # RViz2 for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('isaac_ros_visual_slam'),
        'rviz',
        'default.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        camera_type_arg,
        perception_container,
        rviz_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
