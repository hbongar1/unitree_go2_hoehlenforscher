"""
Loch-Erkennung mittels Voxel Grid (Ansatz 5)
Erkennt Löcher durch Dichte-Gradienten in einem 3D-Voxel-Gitter.
"""

import rclpy
from typing import List, Optional, Tuple
import numpy as np
import struct
from collections import deque, defaultdict
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from go2_my_nodes_py.base_node import BaseNode
from sensor_msgs.msg import PointCloud2, PointField
from go2_msgs.msg import Entrance
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class HoleDetectionVoxelGridNode(BaseNode):
    """
    Erkennt Eingänge durch Voxel-Gitter Dichte-Analyse.
    
    Logik:
    - Teilt Punkte in 3D-Voxel-Gitter ein
    - Berechnet Dichte (Punkte pro Voxel)
    - Analysiert Dichte-Gradienten (Sprünge)
    - Zusammenhängende niedrige Dichte-Regionen = Löcher
    """
    
    def __init__(self):
        super().__init__(
            name="hole_detection_voxel_grid_node",
            description="Erkennt Eingänge durch Voxel Grid Dichte-Analyse"
        )

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_deskewed',
            self.cloud_data_callback,
            qos_profile
        )

        # Publisher
        self.entrance_pub = self.create_publisher(Entrance, 'detected_entrances_voxel_grid', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'entrance_markers_voxel_grid', 10)
        self.cluster_marker_pub = self.create_publisher(MarkerArray, 'cluster_markers_voxel_grid', 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, 'filtered_cloud_voxel_grid', 10)
        
        # Parameter aus aktuellen Thresholds
        self.height_threshold = 0.2
        self.width_threshold = 0.2
        self.max_width = 1.0
        self.cluster_distance = 0.5
        
        # Voxel Grid spezifische Parameter
        self.voxel_size = 0.05  # 5cm Voxel-Größe (adaptive Option verfügbar)
        self.gradient_percentile = 75  # Top 25% Gradienten = Kanten
        self.low_density_percentile = 25  # Unten 25% Dichte = Loch-Kandidaten
        self.gaussian_sigma = 1.0  # Glättungs-Parameter
        
        # Multi-Frame-Akkumulation
        self.frame_buffer_size = 20
        self.point_buffer = deque(maxlen=self.frame_buffer_size)
        self.frame_counter = 0
        self.process_every_n_frames = 5
        
        # Konfidenz-Tracking
        self.entrance_history = {}
        self.confidence_threshold = 2
        self.entrance_timeout = 40
        self.next_entrance_id = 0
        
        self.get_logger().info(
            f"Voxel Grid Node initialisiert:\n"
            f"  - Voxel-Größe: {self.voxel_size}m\n"
            f"  - Gradient-Perzentil: {self.gradient_percentile}%\n"
            f"  - Low-Dichte-Perzentil: {self.low_density_percentile}%"
        )
    
    def cloud_data_callback(self, msg: PointCloud2):
        """Callback für PointCloud2 Daten"""
        self.frame_counter += 1
        
        try:
            points = self.extract_points_from_cloud(msg)
            if points is None or len(points) == 0:
                return
            
            filtered_points = self.filter_points(points)
            if len(filtered_points) == 0:
                return
            
            self.point_buffer.append(filtered_points)
            
            if self.frame_counter % self.process_every_n_frames != 0:
                return
            
            if len(self.point_buffer) < 3:
                self.get_logger().info("Warte auf mehr Frames...")
                return
            
            combined_points = np.vstack(list(self.point_buffer))
            
            self.publish_filtered_cloud(combined_points, msg.header)
            
            clusters = self.cluster_points(combined_points)
            self.get_logger().info(f"[Voxel Grid] Gefundene Cluster: {len(clusters)}")
            
            self.publish_cluster_markers(clusters, msg.header)
            
            current_entrances = []
            for i, cluster in enumerate(clusters):
                entrance = self.analyze_cluster_with_voxel_grid(cluster, msg.header)
                if entrance:
                    current_entrances.append(entrance)
                    self.get_logger().debug(
                        f"Cluster {i}: Eingangs-Kandidat (Voxel Grid-basiert) bei "
                        f"({entrance.position.x:.2f}, {entrance.position.y:.2f})"
                    )
            
            self.update_entrance_tracking(current_entrances)
            self.publish_stable_entrances(msg.header)
            self.publish_entrance_markers(msg.header)
        
        except Exception as e:
            self.get_logger().error(f"[Voxel Grid] Fehler: {e}")
            import traceback
            traceback.print_exc()
    
    def extract_points_from_cloud(self, cloud: PointCloud2) -> Optional[np.ndarray]:
        """Extrahiert XYZ-Punkte"""
        data = np.frombuffer(cloud.data, dtype=np.uint8)
        point_step = cloud.point_step
        
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
        """Filtert Punkte"""
        valid_mask = np.all(np.isfinite(points), axis=1)
        points = points[valid_mask]
        
        if len(points) == 0:
            return points
        
        x_range = np.abs(points[:, 0]) < 5.0
        y_range = np.abs(points[:, 1]) < 5.0
        z_range = (points[:, 2] > 0.0) & (points[:, 2] < 3.0)
        
        valid_mask = x_range & y_range & z_range
        return points[valid_mask]
    
    def cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """DBSCAN-ähnliches Clustering"""
        if len(points) < 10:
            return []
        
        clusters = []
        used = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if used[i]:
                continue
            
            current_cluster = [i]
            used[i] = True
            
            j = 0
            while j < len(current_cluster):
                point_idx = current_cluster[j]
                point = points[point_idx]
                
                distances = np.linalg.norm(points - point, axis=1)
                neighbors = np.where((distances < self.cluster_distance) & (~used))[0]
                
                for neighbor_idx in neighbors:
                    if not used[neighbor_idx]:
                        current_cluster.append(neighbor_idx)
                        used[neighbor_idx] = True
                
                j += 1
            
            if len(current_cluster) > 20:
                clusters.append(points[current_cluster])
        
        return clusters
    
    def create_voxel_grid(self, cluster: np.ndarray) -> Tuple[dict, np.ndarray, np.ndarray]:
        """
        Erstellt Voxel-Gitter aus Cluster-Punkten.
        
        Returns:
            voxel_counts: Dict mit Punkte pro Voxel
            min_bounds: Minimale Grenzen
            grid_shape: Form des Gitters
        """
        min_bounds = np.min(cluster, axis=0)
        max_bounds = np.max(cluster, axis=0)
        
        # Voxel-Indizes berechnen
        voxel_indices = ((cluster - min_bounds) / self.voxel_size).astype(int)
        
        # Zähle Punkte pro Voxel
        voxel_counts = defaultdict(int)
        for idx in voxel_indices:
            key = tuple(idx)
            voxel_counts[key] += 1
        
        # Gitter-Form
        max_idx = np.max(voxel_indices, axis=0)
        grid_shape = max_idx + 1
        
        return voxel_counts, min_bounds, grid_shape
    
    def compute_voxel_density_grid(
        self, cluster: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Berechnet Dichte-Gitter mit Glättung und Gradienten.
        
        Returns:
            density_grid: Dichte pro Voxel
            smoothed_grid: Geglättete Dichte
            gradient_magnitude: Dichte-Gradienten (Kanten)
        """
        try:
            from scipy.ndimage import gaussian_filter, label
        except ImportError:
            self.get_logger().error("scipy wird benötigt für Voxel Grid Analyse")
            return None, None, None
        
        voxel_counts, min_bounds, grid_shape = self.create_voxel_grid(cluster)
        
        # Dichte-Gitter aufbauen
        density_grid = np.zeros(grid_shape, dtype=np.float32)
        for (x, y, z), count in voxel_counts.items():
            if 0 <= x < grid_shape[0] and 0 <= y < grid_shape[1] and 0 <= z < grid_shape[2]:
                density_grid[x, y, z] = count
        
        # Glättung (Rauschen reduzieren)
        smoothed_grid = gaussian_filter(density_grid.astype(float), sigma=self.gaussian_sigma)
        
        # Gradienten berechnen (Dichte-Sprünge)
        dz_dx = np.gradient(smoothed_grid, axis=0)
        dz_dy = np.gradient(smoothed_grid, axis=1)
        dz_dz = np.gradient(smoothed_grid, axis=2)
        
        gradient_magnitude = np.sqrt(dz_dx**2 + dz_dy**2 + dz_dz**2)
        
        return density_grid, smoothed_grid, gradient_magnitude
    
    def detect_hole_regions(self, cluster: np.ndarray) -> bool:
        """
        Detektiert Loch-Regionen durch niedrige Dichte-Bereiche.
        """
        try:
            from scipy.ndimage import label
        except ImportError:
            return False
        
        result = self.compute_voxel_density_grid(cluster)
        if result[0] is None:
            return False
        
        density_grid, smoothed_grid, gradient_magnitude = result
        
        # Schwellwert für niedrige Dichte
        low_density_threshold = np.percentile(
            smoothed_grid[smoothed_grid > 0], 
            self.low_density_percentile
        ) if np.any(smoothed_grid > 0) else 0
        
        low_density_mask = smoothed_grid < low_density_threshold
        
        # Zusammenhängende Komponenten finden
        labeled_array, num_features = label(low_density_mask)
        
        has_hole = num_features > 0
        
        self.get_logger().debug(
            f"Voxel-Gitter Statistik:\n"
            f"  - Grid-Shape: {density_grid.shape}\n"
            f"  - Dichte-Komponenten: {num_features}\n"
            f"  - Niedrig-Dichte-Schwellwert: {low_density_threshold:.2f}\n"
            f"  - Loch erkannt: {has_hole}"
        )
        
        return has_hole
    
    def analyze_cluster_with_voxel_grid(
        self, cluster: np.ndarray, header: Header
    ) -> Optional[Entrance]:
        """
        Analysiert Cluster mit Voxel Grid Methode.
        """
        # Grund-Merkmale
        z_min, z_max = np.min(cluster[:, 2]), np.max(cluster[:, 2])
        x_min, x_max = np.min(cluster[:, 0]), np.max(cluster[:, 0])
        y_min, y_max = np.min(cluster[:, 1]), np.max(cluster[:, 1])
        
        height = z_max - z_min
        x_range = x_max - x_min
        y_range = y_max - y_min
        width = max(x_range, y_range)
        
        # Grund-Filter
        if height < self.height_threshold or width < self.width_threshold or width > self.max_width:
            return None
        
        # Voxel Grid Analyse
        if not self.detect_hole_regions(cluster):
            return None
        
        # Position
        centroid = np.mean(cluster, axis=0)
        
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
        """Konfidenz-Tracking"""
        for entrance_id in list(self.entrance_history.keys()):
            self.entrance_history[entrance_id]['frames_since_seen'] += 1
            if self.entrance_history[entrance_id]['frames_since_seen'] > self.entrance_timeout:
                del self.entrance_history[entrance_id]
        
        for entrance in current_entrances:
            pos = entrance.position
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
        """Publisht Eingänge"""
        published_count = 0
        for entrance_id, data in self.entrance_history.items():
            if data['confidence'] >= self.confidence_threshold:
                entrance = data['entrance']
                entrance.header = header
                self.entrance_pub.publish(entrance)
                published_count += 1
        
        if published_count > 0:
            self.get_logger().info(
                f"[Voxel Grid] {published_count} stabile Eingänge gepublisht"
            )
    
    def publish_filtered_cloud(self, points: np.ndarray, header: Header):
        """Publisht gefilterte Cloud"""
        if len(points) == 0:
            return
        
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True
        
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * len(points)
        
        cloud_msg.data = points.astype(np.float32).tobytes()
        self.filtered_cloud_pub.publish(cloud_msg)
    
    def publish_cluster_markers(self, clusters: List[np.ndarray], header: Header):
        """Visualisiert Cluster"""
        marker_array = MarkerArray()
        
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header = header
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            centroid = np.mean(cluster, axis=0)
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
        
        self.cluster_marker_pub.publish(marker_array)
    
    def publish_entrance_markers(self, header: Header):
        """Visualisiert Eingänge"""
        marker_array = MarkerArray()
        
        for entrance_id, data in self.entrance_history.items():
            if data['confidence'] >= self.confidence_threshold:
                entrance = data['entrance']
                
                marker = Marker()
                marker.header = header
                marker.id = entrance_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                marker.pose.position = entrance.position
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = entrance.width
                marker.scale.y = 0.1
                marker.scale.z = entrance.height
                
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.7
                
                marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = HoleDetectionVoxelGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
