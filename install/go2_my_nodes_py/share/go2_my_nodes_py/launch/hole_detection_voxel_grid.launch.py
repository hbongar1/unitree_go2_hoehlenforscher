from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Startet die Voxel Grid Methode mit RViz Visualisierung"""
    
    # RViz Config Pfad
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('go2_my_nodes_py'),
        'rviz',
        'hole_detection.rviz'
    ])
    
    return LaunchDescription([
        # Voxel Grid Node
        Node(
            package='go2_my_nodes_py',
            executable='hole_detection_voxel_grid',
            name='hole_detection_voxel_grid_node',
            output='screen',
            parameters=[],
        ),
        
        # # RViz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_file],
        # ),
    ])
