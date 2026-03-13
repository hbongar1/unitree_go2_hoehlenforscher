#!/usr/bin/env python3
"""
Launch file for the RANSAC hole detection node with optional visualization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RANSAC hole detection."""
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    hole_detection_node = Node(
        package='go2_my_nodes_py',
        executable='hole_detection_ransac',
        name='hole_detection_ransac_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', 'share/go2_my_nodes_py/rviz/hole_detection.rviz'],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz,
        hole_detection_node,
        rviz_node,
    ])
