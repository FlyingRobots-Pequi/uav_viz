#!/usr/bin/env python3
"""
UAV Monitoring System Launch File

Lança o sistema completo de monitoramento e visualização UAV incluindo:
- Visualizador principal UAV
- RViz com configuração personalizada
- Opções para diferentes modos de operação

Author: UAV Team
License: MIT
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for UAV monitoring system"""
    
    # Package directory
    pkg_share = FindPackageShare('uav_viz')
    
    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'uav_monitoring.rviz']),
            description='Path to RViz configuration file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'uav_viz_params.yaml']),
            description='Path to parameters configuration file'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for all nodes'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for nodes (debug, info, warn, error)'
        ),
        DeclareLaunchArgument(
            'vehicle_frame',
            default_value='base_link',
            description='Vehicle frame ID'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='map',
            description='World/map frame ID'
        ),
        DeclareLaunchArgument(
            'uav_namespace',
            default_value='',
            description='Namespace prefix for all UAV topics (both PX4 and custom)'
        ),
    ]
    
    return LaunchDescription(launch_arguments + [
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    """Setup launch based on arguments"""
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    vehicle_frame = LaunchConfiguration('vehicle_frame').perform(context)
    world_frame = LaunchConfiguration('world_frame').perform(context)
    uav_namespace = LaunchConfiguration('uav_namespace').perform(context)
    
    # Convert string to boolean
    launch_rviz_bool = launch_rviz.lower() in ['true', '1', 'yes']
    use_sim_time_bool = use_sim_time.lower() in ['true', '1', 'yes']
    
    # Define namespace prefix
    ns_prefix = f'/{namespace}' if namespace else ''
    
    # Node parameters
    node_params = [
        params_file,
        {
            'use_sim_time': use_sim_time_bool,
            'vehicle_frame': vehicle_frame,
            'base_frame': world_frame,
            'uav_namespace': uav_namespace,
        }
    ]
    
    # UAV Visualizer Node
    uav_visualizer_node = Node(
        package='uav_viz',
        executable='uav_visualizer',
        name='uav_visualizer',
        namespace=namespace,
        parameters=node_params,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # Static transform publisher for coordinate frames
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        namespace=namespace,
        arguments=['0', '0', '0', '0', '0', '0', world_frame, 'odom'],
        parameters=[{'use_sim_time': use_sim_time_bool}]
    )
    
    # Create node list
    nodes = [
        uav_visualizer_node,
        static_tf_node,
    ]
    
    # Add RViz if requested
    if launch_rviz_bool:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace='',  # RViz typically runs without namespace
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time_bool}],
            output='screen'
        )
        nodes.append(rviz_node)
    
    return nodes


# Alternative launch functions for specific scenarios

def generate_sim_launch_description():
    """Generate launch description optimized for simulation"""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='uav1'),
        DeclareLaunchArgument('log_level', default_value='debug'),
        OpaqueFunction(function=launch_setup)
    ])


def generate_hardware_launch_description():
    """Generate launch description optimized for real hardware"""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    # This allows the launch file to be run directly for testing
    from launch import LaunchService
    
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run() 