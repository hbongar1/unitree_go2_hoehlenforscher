#!/usr/bin/env python3
"""
ROS2 Launch File - Entrance Detection System

Starts all nodes for autonomous entrance navigation:
- depth_camera_node: RealSense depth camera publisher
- unitree_lidar_node: Unitree 4D LiDAR L1 publisher
- verarbeitung_node: Entrance detection processor with sensor fusion
- verhalten_node: Decision logic / state machine
- ausfuehrung_node: Motor controller (Unitree GO2)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from pathlib import Path
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = Path(__file__).parent.parent
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run in simulation mode without Unitree robot'
    )
    
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable Unitree LiDAR sensor fusion'
    )
    
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='enp129s0',
        description='Network interface for Unitree robot connection'
    )
    
    # Launch configuration
    simulation_mode = LaunchConfiguration('simulation_mode')
    use_lidar = LaunchConfiguration('use_lidar')
    network_interface = LaunchConfiguration('network_interface')
    
    return LaunchDescription([
        # Arguments
        use_sim_arg,
        use_lidar_arg,
        network_interface_arg,
        
        # Depth Camera Node - RealSense Camera
        Node(
            package='entrance_detection',  # Will be package name after ROS2 build
            executable='depth_camera_node.py',
            name='depth_camera_node',
            output='screen',
            parameters=[{
                'frame_width': 640,
                'frame_height': 480,
                'fps': 30,
                'enable_filters': True,
            }]
        ),
        
        # Unitree LiDAR Node - 4D LiDAR L1
        Node(
            package='entrance_detection',
            executable='unitree_lidar_node.py',
            name='unitree_lidar_node',
            output='screen',
            parameters=[{
                'min_gap_width': 0.7,
                'max_gap_width': 2.5,
                'max_range': 5.0,
                'camera_fov_deg': 69.0,
                'slice_height_min': -0.3,
                'slice_height_max': 0.3,
            }]
        ),
        
        # Verarbeitung Node - Entrance Detection + Sensor Fusion
        Node(
            package='entrance_detection',
            executable='verarbeitung_node.py',
            name='verarbeitung_node',
            output='screen',
            parameters=[{
                'min_entrance_width': 0.7,
                'min_entrance_height': 1.8,
                'max_entrance_width': 2.5,
                'max_detection_range': 5.0,
                'confidence_threshold': 0.6,
                'safety_clearance': 0.10,
                # Sensor Fusion
                'use_lidar_fusion': True,
                'camera_weight': 0.6,
                'lidar_weight': 0.4,
                'fusion_angle_tolerance': 30.0,
                'fusion_distance_tolerance': 0.5,
            }]
        ),
        
        # Verhalten Node - State Machine / Decision Logic
        Node(
            package='entrance_detection',
            executable='verhalten_node.py',
            name='verhalten_node',
            output='screen',
            parameters=[{
                'normal_height': 0.30,
                'low_height': 0.15,
                'prone_height': 0.08,
                'safety_clearance': 0.10,
                'min_passable_height': 0.10,
            }]
        ),
        
        # Ausfuehrung Node - Motor Control (Unitree GO2)
        Node(
            package='entrance_detection',
            executable='ausfuehrung_node.py',
            name='ausfuehrung_node',
            output='screen',
            parameters=[{
                'network_interface': network_interface,
                'timeout': 10.0,
                'max_height_change_rate': 0.05,
                'simulation_mode': simulation_mode,
            }]
        ),
    ])
