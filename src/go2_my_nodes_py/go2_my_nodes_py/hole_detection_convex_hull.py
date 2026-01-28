"""
Loch-Erkennung mittels Convex Hull (Ansatz 2b)
Erkennt Löcher durch Differenz zwischen Convex Hull und aktuellen Punkten.
"""

import rclpy
from typing import List, Optional
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


class HoleDetectionConvexHullNode(BaseNode):
    """
    Erkennt Eingänge durch Convex Hull Analyse.
    
    Logik:
    - Berechnet Convex Hull (Oberflächenpolyeder) um Cluster
    - Vergleicht Aktualpunkte mit Hull
    - Große Lücken = Löcher/Eingänge
    """
    
    def __init__(self):
        super().__init__(
            name="hole_detection_convex_hull_node",
            description="Erkennt Eingänge durch Convex Hull Differenz-Analyse"
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
        self.entrance_pub = self.create_publisher(Entrance, 'detected_entrances_convex_hull', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'entrance_markers_convex_hull', 10)
        self.cluster_marker_pub = self.create_publisher(MarkerArray, 'cluster_markers_convex_hull', 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, 'filtered_cloud_convex_hull', 10)
        
        # Parameter aus aktuellen Thresholds
        self.height_threshold = 0.2
        self.width_threshold = 0.2
        self.max_width = 1.0
        self.cluster_distance = 0.5
        
        # Convex Hull spezifische Parameter
        self.hole_volume_ratio_threshold = 0.15  # >15% des Volumens = Loch
        self.min_interior_points = 50  # Minimum Punkte für Loch-Detektion
        
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
            f"Convex Hull Node initialisiert:\n"
            f"  - Volumen-Ratio-Schwellwert: {self.hole_volume_ratio_threshold}\n"
            f"  - Min. Punkte für Loch: {self.min_interior_points}"
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
            self.get_logger().info(f"[Convex Hull] Gefundene Cluster: {len(clusters)}")
            
            self.publish_cluster_markers(clusters, msg.header)
            
            current_entrances = []
            for i, cluster in enumerate(clusters):
                entrance = self.analyze_cluster_with_convex_hull(cluster, msg.header)
                if entrance:
                    current_entrances.append(entrance)
                    self.get_logger().debug(
                        f"Cluster {i}: Eingangs-Kandidat (Convex Hull-basiert) bei "
                        f"({entrance.position.x:.2f}, {entrance.position.y:.2f})"
                    )
            
            self.update_entrance_tracking(current_entrances)
            self.publish_stable_entrances(msg.header)
            self.publish_entrance_markers(msg.header)
        
        except Exception as e:
            self.get_logger().error(f"[Convex Hull] Fehler: {e}")
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
        z_range = (points[:, 2] > 0.1) & (points[:, 2] < 3.0)
        
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
    
    def compute_convex_hull(self, cluster: np.ndarray) -> Optional[tuple]:
        """
        Berechnet Convex Hull eines Clusters.
        Gibt Hull-Punkte und innere Punkte zurück.
        """
        if len(cluster) < 10:
            return None
        
        try:
            from scipy.spatial import ConvexHull
            
            # Hull berechnen
            hull = ConvexHull(cluster)
            hull_vertices = cluster[hull.vertices]
            
            return hull, hull_vertices
        
        except Exception as e:
            self.get_logger().warn(f"Convex Hull Berechnung fehlgeschlagen: {e}")
            return None
    
    def analyze_cluster_with_convex_hull(
        self, cluster: np.ndarray, header: Header
    ) -> Optional[Entrance]:
        """
        Analysiert Cluster mit Convex Hull.
        Große Volumen-Differenz deutet auf Loch hin.
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
        
        # Convex Hull Analyse
        hull_result = self.compute_convex_hull(cluster)
        if hull_result is None:
            return None
        
        hull, hull_vertices = hull_result
        
        # Hull-Volumen
        hull_volume = hull.volume
        
        # Bbox Volumen als Näherung für Cluster-Volumen
        bbox_volume = x_range * y_range * height
        
        # Fehlender Volumen-Anteil (potentieller Loch-Indikator)
        if bbox_volume > 0:
            missing_volume_ratio = 1.0 - (len(cluster) / (bbox_volume / 0.01))  # 0.01 = angenommene Punkt-Dichte
            missing_volume_ratio = np.clip(missing_volume_ratio, 0, 1)
        else:
            missing_volume_ratio = 0
        
        # Innere Punkte zählen (nicht auf Hull)
        hull_point_indices = set(hull.vertices)
        interior_point_count = len(cluster) - len(hull_point_indices)
        
        self.get_logger().debug(
            f"Hull-Volumen: {hull_volume:.6f}, Innere Punkte: {interior_point_count}, "
            f"Fehlender Volumen-Anteil: {missing_volume_ratio:.3f}"
        )
        
        # Loch erkannt wenn genug innere Punkte und hoher Volumen-Anteil fehlt
        if interior_point_count < self.min_interior_points:
            return None
        
        if missing_volume_ratio < self.hole_volume_ratio_threshold:
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
                f"[Convex Hull] {published_count} stabile Eingänge gepublisht"
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
            
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
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
                
                marker.color.r = 1.0
                marker.color.g = 0.647
                marker.color.b = 0.0
                marker.color.a = 0.7
                
                marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = HoleDetectionConvexHullNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
