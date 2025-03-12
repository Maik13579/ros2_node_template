#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Base namespace for nodes and container name as sub-namespace
    base_namespace = LaunchConfiguration('namespace')
    container_name = LaunchConfiguration('container_name')
    # Combined namespace for each component node: <namespace>/<container_name>
    node_namespace = [base_namespace, '/', container_name]

    pkg_share = get_package_share_directory('ros2_cpp_component')
    params_file = os.path.join(pkg_share, 'params', 'params.yaml')

    container = ComposableNodeContainer(
        name=container_name,
        namespace=base_namespace,
        package='rclcpp_components',
        executable='component_container_mt', # Use multi-threaded container
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_cpp_component',
                plugin='demo::Talker',
                name='talker',
                namespace=node_namespace,
                extra_arguments=[{'use_intra_process_comms': True}], # Use intra-process communication
                parameters=[params_file],
                remappings=[
                    ('~/chatter', 'chatter')
                ]
            ),
            ComposableNode(
                package='ros2_cpp_component',
                plugin='demo::Listener',
                name='listener',
                namespace=node_namespace,
                extra_arguments=[{'use_intra_process_comms': True}], # Use intra-process communication
                parameters=[params_file],
                remappings=[
                    ('~/chatter', 'chatter')
                ]
            ),
        ],
        output='screen',
        emulate_tty=True # Force unbuffered output like a real terminal
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='my_robot',
            description='Base namespace for all nodes'
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='component_container',
            description='Name for the container node (also used as sub-namespace for components)'
        ),
        container
    ])
