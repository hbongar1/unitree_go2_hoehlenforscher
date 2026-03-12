#!/usr/bin/env python3
"""
Launch file for the clean hole detection node with optional visualization.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for hole detection."""
    
    # Declare launch arguments
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Hole detection clean node
    hole_detection_node = Node(
        package='go2_my_nodes_py',
        executable='hole_detection_clean',
        name='hole_detection_clean_node',
        output='screen',
        parameters=[
            {
                'voxel_size': 0.003,
                'gaussian_sigma': 0.7,
                'height_threshold': 0.1,
                'width_threshold': 0.1,
            }
        ],
    )
    
    # RViz visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [
            'share/go2_my_nodes_py/rviz/hole_detection.rviz'
        ]],
        condition=LaunchConfiguration('rviz')
    )
    
    return LaunchDescription([
        declare_rviz,
        hole_detection_node,
        rviz_node,
    ])
