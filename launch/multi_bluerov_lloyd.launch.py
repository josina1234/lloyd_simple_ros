#!/usr/bin/env python3
"""
Launch file für mehrere Lloyd Path Planner mit Namespaces
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch für mehrere BlueROV Lloyd Path Planner"""
    
    # Anzahl der BlueROVs
    num_bluerovs = LaunchConfiguration('num_bluerovs', default='3')
    
    # Package directory
    package_dir = get_package_share_directory('lloyd_simple')
    config_file = os.path.join(package_dir, 'config', 'simulation_params.yaml')
    
    # Launch arguments
    declare_num_bluerovs = DeclareLaunchArgument(
        'num_bluerovs',
        default_value='3',
        description='Anzahl der BlueROVs'
    )
    
    # Erstelle Nodes für jeden BlueROV
    bluerov_nodes = []
    
    for i in range(3):  # Statisch für Beispiel, kann dynamisch gemacht werden
        namespace = f'bluerov_{i}'
        
        grouped_node = GroupAction([
            PushRosNamespace(namespace),
            Node(
                package='lloyd_simple',
                executable='lloyd_path_planner',
                name='lloyd_path_planner',
                parameters=[
                    config_file,
                    {'robot_id': i, 'namespace': namespace}
                ],
                output='screen',
                emulate_tty=True,
            )
        ])
        bluerov_nodes.append(grouped_node)
    
    return LaunchDescription([
        declare_num_bluerovs,
        *bluerov_nodes,
    ])