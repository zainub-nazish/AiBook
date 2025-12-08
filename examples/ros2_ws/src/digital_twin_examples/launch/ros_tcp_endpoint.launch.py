"""
ros_tcp_endpoint.launch.py
Module 2: The Digital Twin - Chapter 2

Launch file for starting the ROS TCP Endpoint for Unity-ROS communication.

This launch file:
- Starts the ros_tcp_endpoint node
- Configures the TCP port for Unity connection
- Sets up optional TF broadcasting

Usage:
    ros2 launch digital_twin_examples ros_tcp_endpoint.launch.py
    ros2 launch digital_twin_examples ros_tcp_endpoint.launch.py tcp_port:=10000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for ROS TCP Endpoint."""

    # Declare launch arguments
    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='0.0.0.0',
        description='IP address for TCP endpoint (0.0.0.0 = all interfaces)'
    )

    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='10000',
        description='Port number for TCP endpoint'
    )

    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='0',
        description='ROS 2 domain ID for communication'
    )

    # ROS TCP Endpoint node
    # Note: The actual package name may vary depending on installation
    # Common names: ros_tcp_endpoint, ros2_tcp_endpoint
    ros_tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[{
            'ROS_IP': LaunchConfiguration('tcp_ip'),
            'ROS_TCP_PORT': LaunchConfiguration('tcp_port'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # Alternative: If using ros2_tcp_endpoint package
    # ros_tcp_endpoint_node = Node(
    #     package='ros2_tcp_endpoint',
    #     executable='ros2_tcp_endpoint',
    #     name='ros_tcp_endpoint',
    #     parameters=[{
    #         'tcp_ip': LaunchConfiguration('tcp_ip'),
    #         'tcp_port': LaunchConfiguration('tcp_port'),
    #     }],
    #     output='screen',
    # )

    # Log startup information
    log_startup = LogInfo(
        msg=['Starting ROS TCP Endpoint on port: ', LaunchConfiguration('tcp_port')]
    )

    return LaunchDescription([
        # Arguments
        tcp_ip_arg,
        tcp_port_arg,
        ros_domain_id_arg,

        # Log info
        log_startup,

        # Nodes
        ros_tcp_endpoint_node,
    ])
