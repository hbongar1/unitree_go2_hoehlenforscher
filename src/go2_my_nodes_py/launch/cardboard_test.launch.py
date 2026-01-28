#!/usr/bin/env python3
"""
Launch-Datei für Pappe-Cutout Eingangserkennung Test
Optimiert für Indoor-Tests mit Papp-Aufbauten
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Startet Voxel Grid Detection mit Pappe-optimierten Parametern + RViz"""
    
    # Package Share Verzeichnis
    pkg_share = get_package_share_directory('go2_my_nodes_py')
    
    # Parameter-Datei
    params_file = os.path.join(pkg_share, 'config', 'cardboard_detection_params.yaml')
    
    # RViz Config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('go2_my_nodes_py'),
        'rviz',
        'hole_detection.rviz'
    ])
    
    return LaunchDescription([
        # Voxel Grid Node mit Pappe-Parametern
        Node(
            package='go2_my_nodes_py',
            executable='hole_detection_voxel_grid',
            name='hole_detection_voxel_grid_node',
            output='screen',
            parameters=[params_file],
            remappings=[
                # Falls nötig: Topic-Remapping hier einfügen
            ],
        ),
        
        # RViz2 für Live-Visualisierung
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
        
        # Optional: Entrance Decision Node zum Testen der Gesamt-Pipeline
        # Node(
        #     package='go2_my_nodes_py',
        #     executable='entrance_decision_node',
        #     name='entrance_decision_node',
        #     output='screen',
        #     remappings=[
        #         ('detected_entrances', 'detected_entrances_voxel_grid')
        #     ],
        # ),
    ])
