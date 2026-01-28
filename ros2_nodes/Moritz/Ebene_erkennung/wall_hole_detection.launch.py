#!/usr/bin/env python3
"""
Wall and Hole Detection Launch File for ROS2 Humble

Startet den Wall and Hole Detection Node mit der Unitree LiDAR
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Pfad zum Config-Verzeichnis
    config_dir = os.path.join(
        get_package_share_directory('entrance_detection'),
        'config'
    )
    
    wall_hole_config = os.path.join(config_dir, 'wall_hole_detection_params.yaml')
    
    return LaunchDescription([
        # Wall and Hole Detection Node
        Node(
            package='entrance_detection',
            executable='wall_hole_detection_node.py',
            name='wall_hole_detection',
            output='screen',
            parameters=[wall_hole_config],
            remappings=[
                ('/utlidar/cloud', '/utlidar/cloud'),  # Input vom Unitree LiDAR
                ('/detection/wall_plane', '/detection/wall_plane'),
                ('/detection/hole_points', '/detection/hole_points'),
                ('/detection/wall_hole_markers', '/detection/wall_hole_markers'),
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])
