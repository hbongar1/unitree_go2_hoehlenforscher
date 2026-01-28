"""
Flächenerkennung für Höhleneingangs-Detektion
Erkennt ebene Flächen in Point Clouds als Grundlage für Loch-Erkennung.
"""

import rclpy
from typing import List, Optional, Tuple
import numpy as np
import struct
from collections import deque
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from go2_my_nodes_py.base_node import BaseNode
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class SurfaceDetectionNode(BaseNode):
    """
    Erkennt ebene Flächen in Point Clouds mit RANSAC.
    
    Logik:
    - Akkumuliert mehrere Frames
    - Findet ebene Flächen mit RANSAC Plane Fitting
    - Visualisiert gefundene Flächen
    """
    
    def __init__(self):
        super().__init__(
            name="surface_detection_node",
            description="Erkennt ebene Flächen in Point Clouds"
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
        self.surface_marker_pub = self.create_publisher(MarkerArray, 'detected_surfaces', 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, 'filtered_cloud_surfaces', 10)
        
        # RANSAC Parameter
        self.ransac_iterations = 1000  # Anzahl RANSAC Iterationen
        self.distance_threshold = 0.02  # 2cm Abstand für Inliers
        self.min_inliers = 100  # Mindest-Punkte für gültige Fläche
        self.max_surfaces = 5  # Maximale Anzahl zu findender Flächen
        
        # Multi-Frame-Akkumulation
        self.frame_buffer_size = 20
        self.point_buffer = deque(maxlen=self.frame_buffer_size)
        self.frame_counter = 0
        self.process_every_n_frames = 5
        
        self.get_logger().info(
            f"Surface Detection Node initialisiert:\n"
            f"  - RANSAC Iterationen: {self.ransac_iterations}\n"
            f"  - Distance Threshold: {self.distance_threshold}m\n"
            f"  - Min Inliers: {self.min_inliers}\n"
            f"  - Max Surfaces: {self.max_surfaces}"
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
            
            # Finde Flächen
            surfaces = self.detect_surfaces(combined_points)
            self.get_logger().info(f"[Surfaces] Gefundene Flächen: {len(surfaces)}")
            
            self.publish_surface_markers(surfaces, msg.header)
        
        except Exception as e:
            self.get_logger().error(f"[Surfaces] Fehler: {e}")
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
    
    def fit_plane_ransac(self, points: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Findet beste Ebene mit RANSAC.
        
        Ebenen-Gleichung: ax + by + cz + d = 0
        Normale: [a, b, c] (normalisiert)
        
        Returns:
            plane_model: [a, b, c, d] oder None
            inliers: Boolean-Maske der Inlier-Punkte oder None
        """
        if len(points) < 3:
            return None, None
        
        best_inliers = None
        best_model = None
        best_count = 0
        
        for _ in range(self.ransac_iterations):
            # Wähle 3 zufällige Punkte
            sample_indices = np.random.choice(len(points), 3, replace=False)
            sample_points = points[sample_indices]
            
            # Berechne Ebene durch 3 Punkte
            p1, p2, p3 = sample_points
            
            # Zwei Vektoren in der Ebene
            v1 = p2 - p1
            v2 = p3 - p1
            
            # Normale = Kreuzprodukt
            normal = np.cross(v1, v2)
            
            # Überspringe degenerierte Fälle
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            
            normal = normal / norm  # Normalisieren
            
            # d aus ax + by + cz + d = 0
            d = -np.dot(normal, p1)
            
            plane_model = np.append(normal, d)
            
            # Berechne Abstände aller Punkte zur Ebene
            distances = np.abs(
                points[:, 0] * plane_model[0] +
                points[:, 1] * plane_model[1] +
                points[:, 2] * plane_model[2] +
                plane_model[3]
            )
            
            # Inliers
            inliers = distances < self.distance_threshold
            inlier_count = np.sum(inliers)
            
            if inlier_count > best_count:
                best_count = inlier_count
                best_model = plane_model
                best_inliers = inliers
        
        if best_count < self.min_inliers:
            return None, None
        
        return best_model, best_inliers
    
    def detect_surfaces(self, points: np.ndarray) -> List[dict]:
        """
        Findet mehrere Flächen in der Point Cloud.
        
        Returns:
            Liste von Flächen mit plane_model, inlier_points, centroid, normal
        """
        surfaces = []
        remaining_points = points.copy()
        
        for i in range(self.max_surfaces):
            if len(remaining_points) < self.min_inliers:
                break
            
            plane_model, inliers = self.fit_plane_ransac(remaining_points)
            
            if plane_model is None:
                break
            
            inlier_points = remaining_points[inliers]
            centroid = np.mean(inlier_points, axis=0)
            normal = plane_model[:3]
            
            surfaces.append({
                'plane_model': plane_model,
                'inlier_points': inlier_points,
                'centroid': centroid,
                'normal': normal,
                'num_inliers': len(inlier_points)
            })
            
            self.get_logger().info(
                f"Fläche {i+1}: {len(inlier_points)} Punkte, "
                f"Normale: [{normal[0]:.2f}, {normal[1]:.2f}, {normal[2]:.2f}], "
                f"Zentroid: [{centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f}]"
            )
            
            # Entferne Inlier-Punkte für nächste Iteration
            remaining_points = remaining_points[~inliers]
        
        return surfaces
    
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
    
    def publish_surface_markers(self, surfaces: List[dict], header: Header):
        """Visualisiert gefundene Flächen"""
        marker_array = MarkerArray()
        
        colors = [
            (1.0, 0.0, 0.0),  # Rot
            (0.0, 1.0, 0.0),  # Grün
            (0.0, 0.0, 1.0),  # Blau
            (1.0, 1.0, 0.0),  # Gelb
            (1.0, 0.0, 1.0),  # Magenta
        ]
        
        for i, surface in enumerate(surfaces):
            # Flächen-Punkte als Point Cloud
            marker = Marker()
            marker.header = header
            marker.ns = "surface_points"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            
            color = colors[i % len(colors)]
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.5)
            
            for point in surface['inlier_points']:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = float(point[2])
                marker.points.append(p)
            
            marker_array.markers.append(marker)
            
            # Normale als Pfeil
            arrow = Marker()
            arrow.header = header
            arrow.ns = "surface_normals"
            arrow.id = i + 100
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            centroid = surface['centroid']
            normal = surface['normal']
            
            # Startpunkt
            start = Point()
            start.x = float(centroid[0])
            start.y = float(centroid[1])
            start.z = float(centroid[2])
            
            # Endpunkt (0.3m in Richtung Normale)
            end = Point()
            end.x = float(centroid[0] + normal[0] * 0.3)
            end.y = float(centroid[1] + normal[1] * 0.3)
            end.z = float(centroid[2] + normal[2] * 0.3)
            
            arrow.points = [start, end]
            
            arrow.scale.x = 0.02  # Schaftdurchmesser
            arrow.scale.y = 0.04  # Pfeilkopf-Durchmesser
            
            arrow.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            
            marker_array.markers.append(arrow)
        
        self.surface_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
