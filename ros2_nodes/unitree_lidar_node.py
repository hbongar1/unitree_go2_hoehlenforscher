#!/usr/bin/env python3
"""
ROS2 Unitree 4D LiDAR L1 Sensor Node

Verantwortlichkeit:
- Unitree 4D LiDAR L1 Punktwolke empfangen
- Eingänge in 3D-Daten erkennen
- LidarGap Messages publizieren
"""

import rclpy
from rclpy.node import Node
import numpy as np
from typing import List

# ROS2 Messages
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2

# TODO: Custom message (uncomment after build)
# from entrance_detection_msgs.msg import LidarGap


class UnitreeLidarNode(Node):
    """Unitree 4D LiDAR L1 Node with Gap Detection"""
    
    def __init__(self):
        super().__init__('unitree_lidar_node')
        
        # Subscriber for Unitree LiDAR PointCloud
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.cloud_callback, 10
        )
        
        # Publishers
        # TODO: Uncomment after building custom messages
        # self.gap_pub = self.create_publisher(LidarGap, '/sensor/lidar_gaps', 10)
        
        # Publish LaserScan for visualization (2D slice of 3D cloud)
        self.scan_pub = self.create_publisher(LaserScan, '/sensor/lidar_scan', 10)
        
        # Parameters
        self.declare_parameter('min_gap_width', 0.7)
        self.declare_parameter('max_gap_width', 2.5)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('camera_fov_deg', 69.0)
        self.declare_parameter('slice_height_min', -0.3)  # Bodenhöhe
        self.declare_parameter('slice_height_max', 0.3)   # Bis 30cm hoch
        
        self.min_gap_width = self.get_parameter('min_gap_width').value
        self.max_gap_width = self.get_parameter('max_gap_width').value
        self.max_range = self.get_parameter('max_range').value
        self.camera_fov = self.get_parameter('camera_fov_deg').value
        self.slice_height_min = self.get_parameter('slice_height_min').value
        self.slice_height_max = self.get_parameter('slice_height_max').value
        
        self.scan_count = 0
        self.get_logger().info('Unitree 4D LiDAR L1 Node initialized')
        self.get_logger().info(f'Listening on: /utlidar/cloud')
    
    def cloud_callback(self, msg):
        """Receive and process PointCloud2 from Unitree LiDAR"""
        try:
            # Extract 2D slice at ground level for gap detection
            points_2d = self.extract_2d_slice(msg)
            
            if len(points_2d) < 10:
                return
            
            # Find gaps in 2D slice
            gaps = self.find_gaps_in_2d(points_2d)
            
            # Publish LaserScan for visualization
            self.publish_laser_scan(points_2d, msg.header)
            
            # Log detected gaps
            if gaps and self.scan_count % 10 == 0:
                gaps_in_fov = [g for g in gaps if g['in_camera_fov']]
                self.get_logger().info(
                    f'Scan {self.scan_count}: {len(gaps)} gaps detected, '
                    f'{len(gaps_in_fov)} in camera FOV'
                )
                for gap in gaps_in_fov:
                    self.get_logger().info(
                        f"  Gap: {gap['width']:.2f}m @ {gap['angle']:.1f}° "
                        f"(dist: {gap['distance']:.2f}m)"
                    )
            
            self.scan_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
    
    def extract_2d_slice(self, cloud_msg):
        """Extract 2D slice from 3D point cloud at ground level"""
        points_2d = []
        
        # Read point cloud
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            x, y, z = point[:3]
            
            # Filter by height (z-axis) - only ground level
            if self.slice_height_min <= z <= self.slice_height_max:
                # Calculate distance in XY plane
                dist = np.sqrt(x*x + y*y)
                
                # Filter by range
                if 0.3 < dist < self.max_range:
                    angle = np.arctan2(y, x)
                    points_2d.append({
                        'x': x,
                        'y': y,
                        'dist': dist,
                        'angle': angle
                    })
        
        # Sort by angle
        points_2d.sort(key=lambda p: p['angle'])
        
        return points_2d
    
    def find_gaps_in_2d(self, points_2d):
        """Find gaps in 2D point slice"""
        gaps = []
        
        if len(points_2d) < 2:
            return gaps
        
        for i in range(len(points_2d) - 1):
            p1 = points_2d[i]
            p2 = points_2d[i + 1]
            
            # Distance between consecutive points
            dx = p2['x'] - p1['x']
            dy = p2['y'] - p1['y']
            gap_width = np.sqrt(dx*dx + dy*dy)
            
            # Angle difference
            angle_diff = abs(p2['angle'] - p1['angle'])
            
            # Significant gap?
            if self.min_gap_width < gap_width < self.max_gap_width and angle_diff < np.deg2rad(30):
                # Gap center
                center_x = (p1['x'] + p2['x']) / 2
                center_y = (p1['y'] + p2['y']) / 2
                center_dist = np.sqrt(center_x*center_x + center_y*center_y)
                center_angle = np.arctan2(center_y, center_x)
                center_angle_deg = np.rad2deg(center_angle)
                
                # Check if in camera FOV
                in_fov = self.is_in_camera_fov(center_angle_deg)
                
                gaps.append({
                    'width': gap_width,
                    'center': (center_x, center_y),
                    'distance': center_dist,
                    'angle': center_angle_deg,
                    'in_camera_fov': in_fov,
                    'quality': min(1.0, gap_width / 2.0)
                })
        
        return gaps
    
    def is_in_camera_fov(self, angle_deg):
        """Check if angle is within camera field of view"""
        # Normalize angle to -180 to 180
        angle = (angle_deg + 180) % 360 - 180
        fov_min = -self.camera_fov / 2
        fov_max = self.camera_fov / 2
        return fov_min <= angle <= fov_max
    
    def publish_laser_scan(self, points_2d, header):
        """Publish 2D slice as LaserScan for visualization"""
        msg = LaserScan()
        msg.header = header
        msg.header.frame_id = 'lidar_frame'
        
        msg.angle_min = -np.pi
        msg.angle_max = np.pi
        msg.angle_increment = np.deg2rad(1)
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = self.max_range
        
        # Fill ranges (360 bins)
        ranges = [float('inf')] * 360
        for point in points_2d:
            angle_deg = np.rad2deg(point['angle'])
            idx = int((angle_deg + 180) % 360)
            if 0 <= idx < 360:
                ranges[idx] = min(ranges[idx], point['dist'])
        
        msg.ranges = ranges
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeLidarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
