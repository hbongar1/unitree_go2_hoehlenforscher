#!/usr/bin/env python3
"""
ROS2 Verarbeitung Node - Entrance Detection Processor

Verantwortlichkeit:
- Depth-Daten empfangen
- Eingänge erkennen (Tiefensprünge, vertikale Strukturen)
- 3D-Geometrie berechnen
- EntranceEvent und EntranceState publizieren
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import cv2
import pyrealsense2 as rs

# ROS2 Messages
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Point

# TODO: Custom messages (uncomment after build)
# from entrance_detection_msgs.msg import DepthData, EntranceEvent, EntranceState, LidarGap

from sensor_msgs.msg import LaserScan


class VerarbeitungNode(Node):
    """Entrance Detection Processing Node"""
    
    def __init__(self):
        super().__init__('verarbeitung_node')
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, '/sensor/depth_image', self.depth_callback, 10
        )
        self.color_sub = self.create_subscription(
            Image, '/sensor/color_image', self.color_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/sensor/camera_info', self.info_callback, 10
        )
        
        # LiDAR Subscriber (optional for sensor fusion)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/sensor/lidar_scan', self.lidar_callback, 10
        )
        # TODO: Also subscribe to LidarGap after custom messages built
        # self.lidar_gap_sub = self.create_subscription(
        #     LidarGap, '/sensor/lidar_gaps', self.lidar_gap_callback, 10
        # )
        
        # TODO: Publishers (uncomment after custom messages built)
        # self.entrance_pub = self.create_publisher(EntranceEvent, '/perception/entrance', 10)
        # self.state_pub = self.create_publisher(EntranceState, '/perception/entrance_state', 10)
        
        # Parameters
        self.declare_parameter('min_entrance_width', 0.7)
        self.declare_parameter('min_entrance_height', 1.8)
        self.declare_parameter('max_entrance_width', 2.5)
        self.declare_parameter('max_detection_range', 5.0)
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('safety_clearance', 0.10)
        
        # Sensor Fusion parameters
        self.declare_parameter('use_lidar_fusion', True)
        self.declare_parameter('camera_weight', 0.6)  # 60% camera, 40% lidar
        self.declare_parameter('lidar_weight', 0.4)
        self.declare_parameter('fusion_angle_tolerance', 30.0)  # degrees
        self.declare_parameter('fusion_distance_tolerance', 0.5)  # meters
        
        self.min_entrance_width = self.get_parameter('min_entrance_width').value
        self.min_entrance_height = self.get_parameter('min_entrance_height').value
        self.max_entrance_width = self.get_parameter('max_entrance_width').value
        self.max_detection_range = self.get_parameter('max_detection_range').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.safety_clearance = self.get_parameter('safety_clearance').value
        
        # Fusion parameters
        self.use_lidar_fusion = self.get_parameter('use_lidar_fusion').value
        self.camera_weight = self.get_parameter('camera_weight').value
        self.lidar_weight = self.get_parameter('lidar_weight').value
        self.fusion_angle_tol = self.get_parameter('fusion_angle_tolerance').value
        self.fusion_dist_tol = self.get_parameter('fusion_distance_tolerance').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera parameters
        self.intrinsics = None
        self.depth_scale = 0.001
        
        # Frame buffers
        self.depth_image = None
        self.color_image = None
        
        # LiDAR data buffers
        self.latest_lidar_scan = None
        self.lidar_gaps = []  # Will store detected gaps
        
        self.get_logger().info('Verarbeitung Node initialized')
        self.get_logger().info(f'Min entrance: {self.min_entrance_width}m x {self.min_entrance_height}m')
        if self.use_lidar_fusion:
            self.get_logger().info(f'LiDAR fusion enabled: camera={self.camera_weight}, lidar={self.lidar_weight}')
    
    def info_callback(self, msg):
        """Receive camera intrinsics"""
        if self.intrinsics is None:
            # Convert CameraInfo to RealSense intrinsics format
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]  # cx
            self.intrinsics.ppy = msg.k[5]  # cy
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs.distortion.brown_conrady
            self.intrinsics.coeffs = msg.d[:5] if len(msg.d) >= 5 else [0, 0, 0, 0, 0]
            
            self.get_logger().info('Camera intrinsics received')
    
    def depth_callback(self, msg):
        """Receive depth image"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def lidar_callback(self, msg):
        """Receive LiDAR scan data"""
        self.latest_lidar_scan = msg
        # Extract gaps from LaserScan
        self.lidar_gaps = self.extract_gaps_from_scan(msg)
    
    def color_callback(self, msg):
        """Receive color image and trigger processing"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process frame if all data available
            if self.depth_image is not None and self.intrinsics is not None:
                self.process_frame()
                
        except Exception as e:
            self.get_logger().error(f'Color conversion error: {e}')
    
    def process_frame(self):
        """Main processing pipeline"""
        try:
            # Detect depth discontinuities
            discontinuities = self.detect_depth_discontinuities(self.depth_image)
            
            # Detect vertical structures
            gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
            vertical_lines = self.detect_vertical_structures(gray)
            
            # Find entrance candidates
            candidates = self.find_entrance_candidates(
                self.depth_image, vertical_lines
            )
            
            # Validate and publish entrances
            for candidate in candidates:
                entrance_info = self.calculate_entrance_dimensions(candidate)
                
                if entrance_info:
                    is_valid, confidence = self.validate_entrance(entrance_info)
                    
                    # Apply LiDAR fusion if enabled
                    if self.use_lidar_fusion and self.lidar_gaps:
                        confidence = self.fuse_with_lidar(entrance_info, confidence)
                    
                    if is_valid and confidence >= self.confidence_threshold:
                        self.publish_entrance(entrance_info, confidence)
                        # Only process first valid entrance
                        break
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
    
    def detect_depth_discontinuities(self, depth_image):
        """Detect depth jumps using Sobel gradients"""
        # Sobel filters
        sobelx = cv2.Sobel(depth_image, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(depth_image, cv2.CV_64F, 0, 1, ksize=5)
        
        # Gradient magnitude
        gradient = np.sqrt(sobelx**2 + sobely**2)
        gradient_norm = cv2.normalize(gradient, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        # Threshold
        _, discontinuities = cv2.threshold(gradient_norm, 30, 255, cv2.THRESH_BINARY)
        
        return discontinuities
    
    def detect_vertical_structures(self, gray_image):
        """Detect vertical lines (door frames) using Hough Transform"""
        # Edge detection
        edges = cv2.Canny(gray_image, 50, 150)
        
        # Hough Line Transform
        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi/180, threshold=50,
            minLineLength=100, maxLineGap=20
        )
        
        vertical_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.abs(np.arctan2(y2-y1, x2-x1) * 180 / np.pi)
                length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                
                # Filter for vertical lines (85-95 degrees)
                if 85 < angle < 95 and length > 100:
                    vertical_lines.append({
                        'line': (x1, y1, x2, y2),
                        'length': length,
                        'x_pos': (x1 + x2) / 2
                    })
        
        return vertical_lines
    
    def find_entrance_candidates(self, depth_image, vertical_lines):
        """Find entrance candidates from vertical line pairs"""
        if len(vertical_lines) < 2:
            return []
        
        # Sort by x position
        vertical_lines.sort(key=lambda l: l['x_pos'])
        
        candidates = []
        for i in range(len(vertical_lines) - 1):
            left_line = vertical_lines[i]
            right_line = vertical_lines[i + 1]
            
            x_distance = right_line['x_pos'] - left_line['x_pos']
            
            # Filter by pixel width (rough estimate)
            if 80 < x_distance < 400:
                x1 = int(left_line['x_pos'])
                x2 = int(right_line['x_pos'])
                y1 = int(min(left_line['line'][1], right_line['line'][1]))
                y2 = int(max(left_line['line'][3], right_line['line'][3]))
                
                # Bounds check
                h, w = depth_image.shape
                if 0 <= x1 < w and 0 <= x2 < w and 0 <= y1 < h and 0 <= y2 < h:
                    candidates.append({
                        'bbox': (x1, y1, x2-x1, y2-y1),
                        'center_x': (x1 + x2) // 2,
                        'center_y': (y1 + y2) // 2
                    })
        
        return candidates
    
    def calculate_entrance_dimensions(self, candidate):
        """Calculate 3D dimensions of entrance candidate"""
        x, y, w, h = candidate['bbox']
        center_x = candidate['center_x']
        center_y = candidate['center_y']
        
        # Get depth at center
        depth_value = self.depth_image[center_y, center_x] * self.depth_scale
        
        if depth_value == 0 or depth_value > self.max_detection_range:
            return None
        
        # Project to 3D
        try:
            left_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [x, center_y], depth_value
            )
            right_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [x+w, center_y], depth_value
            )
            top_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [center_x, y], depth_value
            )
            bottom_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [center_x, y+h], depth_value
            )
            
            # Calculate dimensions
            width_3d = np.linalg.norm(np.array(left_3d) - np.array(right_3d))
            height_3d = np.linalg.norm(np.array(top_3d) - np.array(bottom_3d))
            
            center_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [center_x, center_y], depth_value
            )
            
            return {
                'width': width_3d,
                'height': height_3d,
                'distance': depth_value,
                'center_3d': center_3d,
                'center_2d': (center_x, center_y)
            }
            
        except Exception as e:
            self.get_logger().warn(f'3D projection failed: {e}')
            return None
    
    def validate_entrance(self, info):
        """Validate entrance and calculate confidence"""
        width = info['width']
        height = info['height']
        distance = info['distance']
        
        confidence = 0.0
        
        # Width check (40%)
        if self.min_entrance_width <= width <= self.max_entrance_width:
            confidence += 0.4
        
        # Height check (30%)
        if height >= self.min_entrance_height:
            confidence += 0.3
        
        # Distance check (20%)
        if 0.5 < distance < self.max_detection_range:
            confidence += 0.2
        
        # Aspect ratio check (10%)
        aspect_ratio = height / width if width > 0 else 0
        if 1.5 < aspect_ratio < 4.0:
            confidence += 0.1
        
        is_valid = confidence > 0.5
        return is_valid, confidence
    
    def publish_entrance(self, info, confidence):
        """Publish detected entrance"""
        self.get_logger().info(
            f"Entrance detected: "
            f"W={info['width']:.2f}m H={info['height']:.2f}m "
            f"D={info['distance']:.2f}m Conf={confidence:.2%}"
        )
        
        # TODO: Publish EntranceEvent and EntranceState messages
        # (uncomment after building custom messages)
        
        # Calculate required state
        required_height = info['height'] - self.safety_clearance
        
        # Determine state
        if required_height >= 0.25:
            state = "NORMAL"
            action_required = False
        elif 0.15 <= required_height < 0.25:
            state = "NIEDRIG"
            action_required = True
        elif 0.10 <= required_height < 0.15:
            state = "LIEGEND"
            action_required = True
        else:
            state = "BLOCKIERT"
            action_required = True
        
        self.get_logger().info(
            f"State: {state}, Required height: {required_height:.2f}m, "
            f"Action: {'YES' if action_required else 'NO'}"
        )
    
    def extract_gaps_from_scan(self, scan_msg):
        """Extract gaps from LaserScan message"""
        gaps = []
        ranges = scan_msg.ranges
        
        # Find gaps in scan
        for i in range(len(ranges) - 1):
            r1 = ranges[i]
            r2 = ranges[i + 1]
            
            # Skip invalid ranges
            if r1 == float('inf') or r2 == float('inf'):
                continue
            if r1 > self.max_detection_range or r2 > self.max_detection_range:
                continue
            
            # Calculate angle and position
            angle1 = scan_msg.angle_min + i * scan_msg.angle_increment
            angle2 = scan_msg.angle_min + (i + 1) * scan_msg.angle_increment
            
            # Cartesian coordinates
            x1 = r1 * np.cos(angle1)
            y1 = r1 * np.sin(angle1)
            x2 = r2 * np.cos(angle2)
            y2 = r2 * np.sin(angle2)
            
            # Gap width
            gap_width = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # Is this a significant gap?
            if self.min_gap_width < gap_width < self.max_gap_width:
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                center_dist = np.sqrt(center_x**2 + center_y**2)
                center_angle_rad = np.arctan2(center_y, center_x)
                center_angle_deg = np.rad2deg(center_angle_rad)
                
                gaps.append({
                    'width': gap_width,
                    'distance': center_dist,
                    'angle': center_angle_deg,
                    'center': (center_x, center_y)
                })
        
        return gaps
    
    def fuse_with_lidar(self, camera_entrance, camera_confidence):
        """Fuse camera detection with LiDAR data"""
        # Calculate camera entrance angle
        cam_x, cam_y = camera_entrance['center_3d'][0], camera_entrance['center_3d'][1]
        cam_angle = np.rad2deg(np.arctan2(cam_y, cam_x))
        cam_distance = camera_entrance['distance']
        
        # Find best matching LiDAR gap
        best_match_score = 0.0
        
        for gap in self.lidar_gaps:
            # Angle difference
            angle_diff = abs(cam_angle - gap['angle'])
            if angle_diff > 180:
                angle_diff = 360 - angle_diff
            
            # Distance difference
            dist_diff = abs(cam_distance - gap['distance'])
            
            # Matching scores (0-1, higher is better)
            angle_score = max(0, 1 - angle_diff / self.fusion_angle_tol)
            dist_score = max(0, 1 - dist_diff / self.fusion_dist_tol)
            
            # Combined match score
            match_score = (angle_score + dist_score) / 2
            
            if match_score > best_match_score:
                best_match_score = match_score
        
        # Fused confidence: weighted combination
        fused_confidence = (
            camera_confidence * self.camera_weight + 
            best_match_score * self.lidar_weight
        )
        
        if best_match_score > 0.5:
            self.get_logger().info(
                f"LiDAR fusion: match_score={best_match_score:.2f}, "
                f"camera_conf={camera_confidence:.2f} → fused={fused_confidence:.2f}"
            )
        
        return fused_confidence
def main(args=None):
    rclpy.init(args=args)
    node = VerarbeitungNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
