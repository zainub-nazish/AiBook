#!/usr/bin/env python3
"""
Isaac ROS Visual SLAM Demo Launch File.

This launch file demonstrates how to run cuVSLAM (GPU-accelerated Visual SLAM)
with stereo camera input for real-time pose tracking.

Usage:
    ros2 launch isaac_ros vslam_demo.launch.py

Requirements:
    - Isaac ROS packages installed
    - Stereo camera publishing to /left/image_raw and /right/image_raw
    - Camera info on /left/camera_info and /right/camera_info
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for VSLAM demo."""

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Camera topics (adjust based on your camera)
    left_image_topic = '/left/image_raw'
    right_image_topic = '/right/image_raw'
    left_camera_info_topic = '/left/camera_info'
    right_camera_info_topic = '/right/camera_info'

    # Isaac ROS Visual SLAM Node
    # Using ComposableNodeContainer for NITROS zero-copy
    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Rectify left image
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_left',
                namespace='left',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', left_image_topic),
                    ('camera_info', left_camera_info_topic),
                ]
            ),
            # Rectify right image
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_right',
                namespace='right',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', right_image_topic),
                    ('camera_info', right_camera_info_topic),
                ]
            ),
            # Visual SLAM node
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    # Input configuration
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,

                    # Camera parameters (example for 640x480)
                    'image_height': 480,
                    'image_width': 640,

                    # SLAM parameters
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,

                    # IMU integration (optional)
                    'enable_imu_fusion': False,

                    # Output frames
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',

                    # Performance tuning
                    'num_cameras': 2,
                    'enable_debug_mode': False,
                }],
                remappings=[
                    ('stereo_camera/left/image', '/left/image_rect'),
                    ('stereo_camera/left/camera_info', '/left/camera_info_rect'),
                    ('stereo_camera/right/image', '/right/image_rect'),
                    ('stereo_camera/right/camera_info', '/right/camera_info_rect'),
                    ('visual_slam/tracking/odometry', '/visual_slam/tracking/odometry'),
                    ('visual_slam/tracking/slam_path', '/visual_slam/tracking/slam_path'),
                ]
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # Static transform: base_link to camera
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0', '0', '0.5',  # x, y, z translation
            '0', '0', '0', '1',  # quaternion (no rotation)
            'base_link', 'camera_link'
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        vslam_container,
        static_tf,
    ])


if __name__ == '__main__':
    generate_launch_description()
