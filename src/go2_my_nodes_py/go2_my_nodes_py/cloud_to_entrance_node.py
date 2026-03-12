import rclpy
from typing import List, Dict
import numpy as np
import struct
from collections import deque
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from go2_my_nodes_py.base_node import BaseNode
from sensor_msgs.msg import PointCloud2, PointField
from go2_msgs.msg import Entrance
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class CloudToEntranceNode(BaseNode):
    """Node for processing PointCloud2 into Entrance detections with multi-frame accumulation"""
    
    def __init__(self):
        super().__init__(
            name="cloud_to_entrance_node",
            description="Processes PointCloud2 into Entrance candidates with temporal accumulation"
        )

        # Add subscriber with flexible QoS
        # Try Reliable first, then Best-Effort if that fails
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Not critical for sensors
            durability=QoSDurabilityPolicy.VOLATILE  # PointCloud is volatile
        )
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_deskewed',
            self.cloud_data_callback,
            qos_profile
        )

        # Publishers
        self.publisher = self.create_publisher(Entrance, 'detected_entrances', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'entrance_markers', 10)
        self.cluster_marker_pub = self.create_publisher(MarkerArray, 'cluster_markers', 10)
        
        # Detection thresholds
        self.height_threshold = 0.2      # Minimum entrance height in meters
        self.width_threshold = 0.2       # Minimum entrance width in meters
        self.max_width = 1.0             # Maximum entrance width in meters
        self.cluster_distance = 0.5      # Point clustering radius in meters
        
        # Multi-frame accumulation
        self.frame_buffer_size = 20      # Number of frames to accumulate
        self.point_buffer = deque(maxlen=self.frame_buffer_size)
        self.frame_counter = 0
        self.process_every_n_frames = 5  # Only process every n frames
        
        # Entrance tracking across frames
        self.entrance_history: Dict[int, dict] = {}
        self.confidence_threshold = 2    # Minimum detections to publish
        self.entrance_timeout = 40       # Frames before removal
        self.next_entrance_id = 0
        
        self.get_logger().info(
            f"Cloud-to-Entrance node initialized: "
            f"accumulating {self.frame_buffer_size} frames, "
            f"processing every {self.process_every_n_frames} frame(s), "
            f"min confidence for publishing: {self.confidence_threshold}"
        )
    
    def cloud_data_callback(self, msg: PointCloud2):
        """Process PointCloud with multi-frame accumulation and entrance detection"""
        self.frame_counter += 1
        
        try:
            # 1. Extract points from current PointCloud
            points = self.extract_points_from_cloud(msg)
            
            if points is None or len(points) == 0:
                return
            
            # 2. Filter points
            filtered_points = self.filter_points(points)
            
            if len(filtered_points) == 0:
                return
            
            # Add to frame buffer
            self.point_buffer.append(filtered_points)
            
            # Process every N frames for efficiency
            if self.frame_counter % self.process_every_n_frames != 0:
                return
            
            if len(self.point_buffer) < 3:
                return
            
            # Combine accumulated points from buffer
            combined_points = np.vstack(list(self.point_buffer))
            
            # Detect clusters and create entrance candidates
            clusters = self.cluster_points(combined_points)
            current_entrances = []
            for cluster in clusters:
                entrance = self.analyze_cluster_for_entrance(cluster, msg.header)
                if entrance:
                    current_entrances.append(entrance)
            
            # Visualize clusters
            self.publish_cluster_markers(clusters, msg.header)
            
            # Track entrances and publish stable detections
            self.update_entrance_tracking(current_entrances)
            self.publish_stable_entrances(msg.header)
            self.publish_entrance_markers(msg.header)
        
        except Exception as e:
            self.get_logger().error(f"Error in cloud processing: {e}")
            import traceback
            traceback.print_exc()
    
    def extract_points_from_cloud(self, cloud: PointCloud2) -> np.ndarray:
        """
        Extracts XYZ points from PointCloud2
        
        PointCloud2 stores data as continuous byte array.
        """
        data = np.frombuffer(cloud.data, dtype=np.uint8)
        point_step = cloud.point_step
        
        # Extract offset for X, Y, Z from fields
        x_offset = y_offset = z_offset = None
        for field in cloud.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if None in (x_offset, y_offset, z_offset):
            return None
        
        # Read points
        point_count = cloud.width * cloud.height
        points = np.zeros((point_count, 3), dtype=np.float32)
        
        for i in range(point_count):
            base_offset = i * point_step
            
            try:
                x_bytes = data[base_offset + x_offset:base_offset + x_offset + 4]
                y_bytes = data[base_offset + y_offset:base_offset + y_offset + 4]
                z_bytes = data[base_offset + z_offset:base_offset + z_offset + 4]
                
                points[i, 0] = struct.unpack('f', x_bytes)[0]
                points[i, 1] = struct.unpack('f', y_bytes)[0]
                points[i, 2] = struct.unpack('f', z_bytes)[0]
            except:
                points[i] = [np.nan, np.nan, np.nan]
        
        return points
    
    def filter_points(self, points: np.ndarray) -> np.ndarray:
        """Remove invalid points and limit to relevant detection range"""
        # Remove NaN/Inf
        valid_mask = np.all(np.isfinite(points), axis=1)
        points = points[valid_mask]
        
        if len(points) == 0:
            return points
        
        # Keep only points within 5m horizontal radius, 0.1-3m height
        x_range = np.abs(points[:, 0]) < 5.0
        y_range = np.abs(points[:, 1]) < 5.0
        z_range = (points[:, 2] > 0.1) & (points[:, 2] < 3.0)
        
        valid_mask = x_range & y_range & z_range
        return points[valid_mask]
    
    def cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """Group spatially nearby points into clusters for entrance analysis"""
        if len(points) < 10:
            return []
        
        clusters = []
        used = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if used[i]:
                continue
            
            # Start new cluster
            current_cluster = [i]
            used[i] = True
            
            # Expand neighbors (BFS-like)
            j = 0
            while j < len(current_cluster):
                point_idx = current_cluster[j]
                point = points[point_idx]
                
                # Distances to all other points
                distances = np.linalg.norm(points - point, axis=1)
                
                # Find neighbors within radius
                neighbors = np.where(
                    (distances < self.cluster_distance) & (~used)
                )[0]
                
                # Add all new neighbors to cluster
                for neighbor_idx in neighbors:
                    if not used[neighbor_idx]:
                        current_cluster.append(neighbor_idx)
                        used[neighbor_idx] = True
                
                j += 1
            
            # Only keep larger clusters (filter noise)
            if len(current_cluster) > 20:
                clusters.append(points[current_cluster])
        
        return clusters
    
    def analyze_cluster_for_entrance(
        self, cluster: np.ndarray, header: Header
    ) -> Entrance:
        """Check if cluster matches entrance dimensions (height > width)"""
        # Calculate dimensions
        z_min, z_max = np.min(cluster[:, 2]), np.max(cluster[:, 2])
        x_min, x_max = np.min(cluster[:, 0]), np.max(cluster[:, 0])
        y_min, y_max = np.min(cluster[:, 1]), np.max(cluster[:, 1])
        
        height = z_max - z_min
        x_range = x_max - x_min
        y_range = y_max - y_min
        width = max(x_range, y_range)
        
        if height < self.height_threshold or width < self.width_threshold or width > self.max_width:
            return None
        
        # Position (centroid of cluster)
        centroid = np.mean(cluster, axis=0)
        
        # Create Entrance message
        entrance = Entrance()
        entrance.header = header
        entrance.position = Point(
            x=float(centroid[0]),
            y=float(centroid[1]),
            z=float(centroid[2])
        )
        entrance.width = float(width)
        entrance.height = float(height)
        
        return entrance
    
    def update_entrance_tracking(self, current_entrances: List[Entrance]):
        """Track entrances across frames: new=confidence 1, repeated=increase confidence"""
        # Age all existing entrances
        for entrance_id in list(self.entrance_history.keys()):
            self.entrance_history[entrance_id]['frames_since_seen'] += 1
            
            # Remove old entrances
            if self.entrance_history[entrance_id]['frames_since_seen'] > self.entrance_timeout:
                del self.entrance_history[entrance_id]
        
        # Process current entrances
        for entrance in current_entrances:
            pos = entrance.position
            
            # Check if entrance is already known (spatial proximity)
            matched_id = None
            for entrance_id, data in self.entrance_history.items():
                stored_pos = data['position']
                distance = np.sqrt(
                    (pos.x - stored_pos.x)**2 + 
                    (pos.y - stored_pos.y)**2 + 
                    (pos.z - stored_pos.z)**2
                )
                
                if distance < 0.8:
                    matched_id = entrance_id
                    break
            
            if matched_id is not None:
                self.entrance_history[matched_id]['confidence'] += 1
                self.entrance_history[matched_id]['frames_since_seen'] = 0
                self.entrance_history[matched_id]['entrance'] = entrance
            else:
                new_id = self.next_entrance_id
                self.next_entrance_id += 1
                self.entrance_history[new_id] = {
                    'position': pos,
                    'entrance': entrance,
                    'confidence': 1,
                    'frames_since_seen': 0
                }
    
    def publish_stable_entrances(self, header: Header):
        """Publish entrances with sufficient confidence (detected multiple times)"""
        for entrance_id, data in self.entrance_history.items():
            if data['confidence'] >= self.confidence_threshold:
                entrance = data['entrance']
                entrance.header = header
                self.publisher.publish(entrance)
    

    
    def publish_cluster_markers(self, clusters: List[np.ndarray], header: Header):
        """Visualize clusters as colored boxes in RViz"""
        marker_array = MarkerArray()
        
        # Distinct colors for visualization
        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0),  # Cyan
            (1.0, 0.5, 0.0),  # Orange
            (1.0, 0.0, 0.5),  # Pink
            (0.5, 0.0, 1.0),  # Purple
            (0.0, 0.5, 1.0),  # Cyan-Blue
        ]
        
        for cluster_id, cluster in enumerate(clusters):
            # Calculate cluster bounding box
            x_min, x_max = np.min(cluster[:, 0]), np.max(cluster[:, 0])
            y_min, y_max = np.min(cluster[:, 1]), np.max(cluster[:, 1])
            z_min, z_max = np.min(cluster[:, 2]), np.max(cluster[:, 2])
            
            # Centroid
            centroid = np.mean(cluster, axis=0)
            
            # Color from list (cyclic)
            color_idx = cluster_id % len(colors)
            r, g, b = colors[color_idx]
            
            # Create bounding box marker
            marker = Marker()
            marker.header = header
            if not marker.header.frame_id:
                marker.header.frame_id = 'odom'
            marker.ns = "clusters"
            marker.id = cluster_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position (centroid)
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.pose.orientation.w = 1.0
            
            # Size (bounding box extension)
            marker.scale.x = float(x_max - x_min) if x_max > x_min else 0.1
            marker.scale.y = float(y_max - y_min) if y_max > y_min else 0.1
            marker.scale.z = float(z_max - z_min) if z_max > z_min else 0.1
            
            # Color (transparent to keep points visible)
            marker.color = ColorRGBA(r=r, g=g, b=b, a=0.3)
            
            marker.lifetime.sec = 2
            marker_array.markers.append(marker)
            
            # Text label with cluster ID and point count
            text_marker = Marker()
            text_marker.header = header
            if not text_marker.header.frame_id:
                text_marker.header.frame_id = 'odom'
            text_marker.ns = "cluster_labels"
            text_marker.id = cluster_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Text in center of cluster
            text_marker.pose.position.x = float(centroid[0])
            text_marker.pose.position.y = float(centroid[1])
            text_marker.pose.position.z = float(centroid[2])
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.25  # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"C{cluster_id}\n({len(cluster)})"
            text_marker.lifetime.sec = 2
            
            marker_array.markers.append(text_marker)
        
        if len(marker_array.markers) > 0:
            self.cluster_marker_pub.publish(marker_array)
    
    def publish_entrance_markers(self, header: Header):
        """Visualize stable detections as entrance boxes in RViz"""
        marker_array = MarkerArray()
        
        for entrance_id, data in self.entrance_history.items():
            if data['confidence'] >= self.confidence_threshold:
                entrance = data['entrance']
                
                # Create entrance box marker
                marker = Marker()
                marker.header = header
                # Ensure frame_id is set
                if not marker.header.frame_id:
                    marker.header.frame_id = 'odom'
                marker.ns = "entrances"
                marker.id = entrance_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Position
                marker.pose.position = entrance.position
                marker.pose.orientation.w = 1.0
                
                # Size (width, depth=0.1m, height)
                marker.scale.x = float(entrance.width)
                marker.scale.y = 0.1  # Thin wall
                marker.scale.z = float(entrance.height)
                
                # Color (green with transparency, confidence determines brightness)
                alpha = min(1.0, data['confidence'] / 10.0)
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=alpha)
                
                marker.lifetime.sec = 2  # 2 seconds lifetime
                
                marker_array.markers.append(marker)
                
                # Text label with ID and confidence
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.ns = "entrance_labels"
                text_marker.id = entrance_id + 1000  # Offset for text IDs
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                # Text above entrance
                text_marker.pose.position.x = entrance.position.x
                text_marker.pose.position.y = entrance.position.y
                text_marker.pose.position.z = entrance.position.z + entrance.height / 2 + 0.3
                text_marker.pose.orientation.w = 1.0
                
                text_marker.scale.z = 0.3  # Text height
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                text_marker.text = f"E{entrance_id}\nC:{data['confidence']}"
                text_marker.lifetime.sec = 2
                
                marker_array.markers.append(text_marker)
        
        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)
    
def main(args=None):
    rclpy.init(args=args)
    node = CloudToEntranceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

