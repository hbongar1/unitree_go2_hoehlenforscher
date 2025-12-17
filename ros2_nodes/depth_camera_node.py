#!/usr/bin/env python3
"""
ROS2 Depth Camera Node - RealSense Depth Camera Publisher

Verantwortlichkeit:
- RealSense Kamera initialisieren
- Depth + Color Frames erfassen
- Rohdaten auf ROS2 Topics publizieren
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs

# ROS2 Messages
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header


class DepthCameraNode(Node):
    """RealSense Depth Camera Publisher Node"""
    
    def __init__(self):
        super().__init__('depth_camera_node')
        
        # ROS2 Publishers
        self.depth_pub = self.create_publisher(Image, '/sensor/depth_image', 10)
        self.color_pub = self.create_publisher(Image, '/sensor/color_image', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/sensor/camera_info', 10)
        
        # Parameters
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_filters', True)
        
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.enable_filters = self.get_parameter('enable_filters').value
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # RealSense Setup
        self.pipeline = None
        self.depth_scale = 0.001
        self.intrinsics = None
        
        # Filters (optional)
        self.spatial_filter = None
        self.temporal_filter = None
        
        # Initialize camera
        self.initialize_camera()
        
        # Timer for publishing (30 Hz)
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.frame_count = 0
        self.get_logger().info('Depth Camera Node initialized')
    
    def initialize_camera(self):
        """Initialize RealSense camera pipeline"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Configure streams
            config.enable_stream(
                rs.stream.depth,
                self.frame_width,
                self.frame_height,
                rs.format.z16,
                self.fps
            )
            config.enable_stream(
                rs.stream.color,
                self.frame_width,
                self.frame_height,
                rs.format.bgr8,
                self.fps
            )
            
            # Start pipeline
            profile = self.pipeline.start(config)
            
            # Get depth scale
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            # Get camera intrinsics
            depth_stream = profile.get_stream(rs.stream.depth)
            self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
            
            # Setup filters if enabled
            if self.enable_filters:
                self.spatial_filter = rs.spatial_filter()
                self.temporal_filter = rs.temporal_filter()
                self.get_logger().info('Filters enabled')
            
            self.get_logger().info(
                f'RealSense initialized: {self.frame_width}x{self.frame_height} @ {self.fps} Hz'
            )
            self.get_logger().info(f'Depth scale: {self.depth_scale}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')
            raise
    
    def apply_filters(self, depth_frame):
        """Apply RealSense filters to depth frame"""
        if self.enable_filters and self.spatial_filter and self.temporal_filter:
            depth_frame = self.spatial_filter.process(depth_frame)
            depth_frame = self.temporal_filter.process(depth_frame)
        return depth_frame
    
    def publish_frame(self):
        """Capture and publish camera frames"""
        try:
            # Get frames from camera
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                self.get_logger().warn('No frames received')
                return
            
            # Apply filters
            depth_frame = self.apply_filters(depth_frame)
            
            # Convert to numpy
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Create ROS2 message header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_depth_frame'
            
            # Convert to ROS2 Image messages
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
            depth_msg.header = header
            
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            color_msg.header = header
            
            # Create CameraInfo message
            camera_info = self.create_camera_info(header)
            
            # Publish
            self.depth_pub.publish(depth_msg)
            self.color_pub.publish(color_msg)
            self.info_pub.publish(camera_info)
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Log every second
                self.get_logger().info(
                    f'Published frame {self.frame_count} @ {self.fps} Hz'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {e}')
    
    def create_camera_info(self, header):
        """Create CameraInfo message from intrinsics"""
        msg = CameraInfo()
        msg.header = header
        msg.width = self.intrinsics.width
        msg.height = self.intrinsics.height
        msg.distortion_model = 'plumb_bob'
        
        # Intrinsic matrix
        msg.k = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx,
            0.0, self.intrinsics.fy, self.intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix
        msg.p = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx, 0.0,
            0.0, self.intrinsics.fy, self.intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Distortion coefficients
        msg.d = list(self.intrinsics.coeffs)
        
        return msg
    
    def destroy_node(self):
        """Clean shutdown"""
        if self.pipeline:
            self.pipeline.stop()
            self.get_logger().info('Camera pipeline stopped')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
