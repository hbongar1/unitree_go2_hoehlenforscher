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
        self.hole_marker_pub = self.create_publisher(MarkerArray, 'detected_holes', 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, 'filtered_cloud_surfaces', 10)
        
        # RANSAC Parameter
        self.ransac_iterations = 1000  # Anzahl RANSAC Iterationen
        self.distance_threshold = 0.02  # 2cm Abstand für Inliers
        self.min_inliers = 100  # Mindest-Punkte für gültige Fläche
        self.max_surfaces = 5  # Maximale Anzahl zu findender Flächen
        
        # Vertikale Flächen Filter (auf dem Boden stehend)
        self.min_vertical_angle = 60.0  # Mindestens 60° zur Horizontalen (30° zur Vertikalen)
        self.max_ground_height = 0.3  # Fläche muss bis max 30cm über Boden reichen
        
        # Loch-Erkennung in Flächen
        self.hole_grid_size = 0.05  # 5cm Gitter auf der Fläche
        self.hole_min_density_percentile = 30  # Untere 30% Dichte = Loch
        self.hole_min_size = 0.1  # Minimum 10cm² Loch-Größe
        
        # Multi-Frame-Akkumulation
        self.frame_buffer_size = 50  # Mehr Frames für längeren Zeitraum
        self.point_buffer = deque(maxlen=self.frame_buffer_size)
        self.frame_counter = 0
        self.process_every_n_frames = 10  # Seltener verarbeiten für stabilere Flächen
        
        self.get_logger().info(
            f"Surface Detection Node initialisiert:\n"
            f"  - RANSAC Iterationen: {self.ransac_iterations}\n"
            f"  - Distance Threshold: {self.distance_threshold}m\n"
            f"  - Min Inliers: {self.min_inliers}\n"
            f"  - Max Surfaces: {self.max_surfaces}\n"
            f"  - Frame Buffer: {self.frame_buffer_size} Frames\n"
            f"  - Verarbeitung alle: {self.process_every_n_frames} Frames"
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
            
            if len(self.point_buffer) < 10:  # Warte auf mehr Frames
                self.get_logger().info(f"Sammle Daten... ({len(self.point_buffer)}/{self.frame_buffer_size} Frames)")
                return
            
            combined_points = np.vstack(list(self.point_buffer))
            
            self.get_logger().info(f"Verarbeite {len(combined_points)} Punkte aus {len(self.point_buffer)} Frames")
            
            self.publish_filtered_cloud(combined_points, msg.header)
            
            # Finde Flächen
            surfaces = self.detect_surfaces(combined_points)
            self.get_logger().info(f"[Surfaces] Gefundene Flächen: {len(surfaces)}")
            
            # Finde Löcher in Flächen
            holes = []
            for surface in surfaces:
                surface_holes = self.detect_holes_in_surface(surface)
                holes.extend(surface_holes)
            
            self.get_logger().info(f"[Surfaces] Gefundene Löcher: {len(holes)}")
            
            self.publish_surface_markers(surfaces, msg.header)
            self.publish_hole_markers(holes, msg.header)
        
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
        
        x_range = np.abs(points[:, 0]) < 2.0
        y_range = np.abs(points[:, 1]) < 2.0
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
    
    def is_vertical_surface(self, normal: np.ndarray, points: np.ndarray) -> bool:
        """
        Prüft ob Fläche vertikal steht (auf dem Boden).
        
        Args:
            normal: Flächennormale [x, y, z]
            points: Punkte der Fläche
            
        Returns:
            True wenn Fläche vertikal und am Boden
        """
        # Winkel zur Horizontalen (z-Achse zeigt nach oben)
        z_component = abs(normal[2])
        angle_to_horizontal = np.arccos(np.clip(z_component, 0, 1)) * 180 / np.pi
        
        # Fläche muss nahezu vertikal sein (60-90° zur Horizontalen)
        if angle_to_horizontal < self.min_vertical_angle:
            return False
        
        # Prüfe ob Fläche am Boden steht (min z-Wert der Punkte)
        min_height = np.min(points[:, 2])
        if min_height > self.max_ground_height:
            return False
        
        return True
    
    def project_points_to_plane(self, points: np.ndarray, plane_model: np.ndarray) -> np.ndarray:
        """
        Projiziert 3D-Punkte auf eine Ebene und gibt 2D-Koordinaten zurück.
        
        Args:
            points: [N, 3] 3D-Punkte
            plane_model: [a, b, c, d] Ebenen-Modell (ax + by + cz + d = 0)
            
        Returns:
            [N, 2] 2D-Koordinaten auf der Ebene
        """
        normal = plane_model[:3]
        
        # Erstelle orthonormale Basis auf der Ebene
        # u-Achse: senkrecht zu Normale und z-Achse
        z_axis = np.array([0, 0, 1])
        u_axis = np.cross(normal, z_axis)
        u_norm = np.linalg.norm(u_axis)
        if u_norm > 1e-6:
            u_axis = u_axis / u_norm
        else:
            # Fallback wenn Normale parallel zu z-Achse
            u_axis = np.array([1, 0, 0])
        
        # v-Achse: senkrecht zu Normale und u-Achse
        v_axis = np.cross(normal, u_axis)
        v_axis = v_axis / np.linalg.norm(v_axis)
        
        # Projiziere jeden Punkt
        coords_2d = np.zeros((len(points), 2))
        for i, point in enumerate(points):
            coords_2d[i, 0] = np.dot(point, u_axis)
            coords_2d[i, 1] = np.dot(point, v_axis)
        
        return coords_2d
    
    def detect_holes_in_surface(self, surface: dict) -> List[dict]:
        """
        Erkennt Löcher in einer erkannten Fläche durch Densitäts-Analyse.
        Löcher müssen vertikal (orthogonal zum Boden) und INNERHALB der Fläche sein.
        
        Args:
            surface: Flächen-Dict mit plane_model, inlier_points, normal, centroid
            
        Returns:
            Liste von gefundenen Löchern
        """
        points = surface['inlier_points']
        plane_model = surface['plane_model']
        
        # Nur die Punkte dieser Fläche verwenden (bereits gefiltert)
        if len(points) < 10:
            return []
        
        # Projiziere Punkte auf die Ebene
        coords_2d = self.project_points_to_plane(points, plane_model)
        
        # Erstelle 2D-Gitter auf der Ebene
        min_u = np.min(coords_2d[:, 0])
        max_u = np.max(coords_2d[:, 0])
        min_v = np.min(coords_2d[:, 1])
        max_v = np.max(coords_2d[:, 1])
        
        # Gitter-Dimensionen
        grid_u = int(np.ceil((max_u - min_u) / self.hole_grid_size))
        grid_v = int(np.ceil((max_v - min_v) / self.hole_grid_size))
        
        if grid_u < 2 or grid_v < 2:
            return []
        
        # Zähle Punkte pro Gitter-Zelle
        grid = np.zeros((grid_u, grid_v))
        for u, v in coords_2d:
            iu = int((u - min_u) / self.hole_grid_size)
            iv = int((v - min_v) / self.hole_grid_size)
            if 0 <= iu < grid_u and 0 <= iv < grid_v:
                grid[iu, iv] += 1
        
        # Finde niedrig-Dichte Bereiche (Löcher)
        non_zero = grid[grid > 0]
        if len(non_zero) == 0:
            return []
        
        density_threshold = np.percentile(non_zero, self.hole_min_density_percentile)
        hole_mask = (grid < density_threshold) & (grid >= 0)
        
        # Finde zusammenhängende Loch-Regionen (ohne scipy)
        labeled_array = self.label_connected_components(hole_mask)
        
        holes = []
        for hole_id in range(1, np.max(labeled_array) + 1):
            hole_cells = np.argwhere(labeled_array == hole_id)
            
            if len(hole_cells) < 3:  # Mindestgröße
                continue
            
            # Berechne Loch-Zentroid in 2D
            hole_center_u = np.mean(hole_cells[:, 0]) * self.hole_grid_size + min_u
            hole_center_v = np.mean(hole_cells[:, 1]) * self.hole_grid_size + min_v
            
            # Größe des Lochs
            hole_size = len(hole_cells) * (self.hole_grid_size ** 2)
            
            if hole_size < self.hole_min_size:
                continue
            
            # Flächennormale
            surface_normal = surface['normal']
            
            # Loch-Normale ist senkrecht zur Fläche (zeigt orthogonal auf Boden)
            # Falls Fläche vertikal ist, zeigt hole_normal vertikal (z-Richtung)
            hole_normal = surface_normal.copy()
            
            # Transformiere zurück zu 3D
            z_axis = np.array([0, 0, 1])
            u_axis = np.cross(surface_normal, z_axis)
            u_norm = np.linalg.norm(u_axis)
            if u_norm > 1e-6:
                u_axis = u_axis / u_norm
            else:
                u_axis = np.array([1, 0, 0])
            
            v_axis = np.cross(surface_normal, u_axis)
            v_axis = v_axis / np.linalg.norm(v_axis)
            
            # Berechne 3D Position des Loch-Zentrums auf der Fläche
            hole_center_3d = (
                surface['centroid'] +
                u_axis * (hole_center_u - np.dot(surface['centroid'], u_axis)) +
                v_axis * (hole_center_v - np.dot(surface['centroid'], v_axis))
            )
            
            # Validierung: Loch muss innerhalb der Flächen-Grenzen sein
            # Prüfe ob Loch-Zentrum innerhalb der 2D-Projektion liegt
            if not (min_u <= hole_center_u <= max_u and min_v <= hole_center_v <= max_v):
                continue
            
            holes.append({
                'center': hole_center_3d,
                'size': hole_size,
                'num_cells': len(hole_cells),
                'surface_normal': surface_normal,
                'hole_normal': hole_normal,  # Senkrecht auf Boden
                'centroid': surface['centroid']
            })
            
            self.get_logger().info(
                f"Loch gefunden: Größe={hole_size:.4f}m², "
                f"Zentrum=[{hole_center_3d[0]:.2f}, {hole_center_3d[1]:.2f}, {hole_center_3d[2]:.2f}], "
                f"In Fläche mit Normale=[{surface_normal[0]:.2f}, {surface_normal[1]:.2f}, {surface_normal[2]:.2f}]"
            )
        
        return holes
    
    def publish_hole_markers(self, holes: List[dict], header: Header):
        """Visualisiert gefundene Löcher als Markierungen"""
        marker_array = MarkerArray()
        
        for i, hole in enumerate(holes):
            # Loch-Zentrum als Kugel
            sphere = Marker()
            sphere.header = header
            sphere.ns = "hole_centers"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            
            center = hole['center']
            sphere.pose.position.x = float(center[0])
            sphere.pose.position.y = float(center[1])
            sphere.pose.position.z = float(center[2])
            sphere.pose.orientation.w = 1.0
            
            # Größe der Kugel basierend auf Loch-Größe
            sphere_radius = np.sqrt(hole['size'] / np.pi)
            sphere.scale.x = sphere_radius * 2
            sphere.scale.y = sphere_radius * 2
            sphere.scale.z = sphere_radius * 2
            
            sphere.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7)  # Orange
            
            marker_array.markers.append(sphere)
            
            # Fläche im Loch-Bereich als Quadrat
            cube = Marker()
            cube.header = header
            cube.ns = "hole_surfaces"
            cube.id = i + 100
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            
            cube.pose.position.x = float(center[0])
            cube.pose.position.y = float(center[1])
            cube.pose.position.z = float(center[2])
            
            # Orientierung der Fläche (Normale)
            normal = hole['surface_normal']
            hole_normal = hole['hole_normal']
            
            # Berechne Quaternion aus der Loch-Normalen (senkrecht zum Boden)
            # Die z-Komponente zeigt nach oben, hole_normal zeigt in diese Richtung
            cube.pose.orientation.w = 1.0
            
            # Größe des Quadrats
            size = np.sqrt(hole['size'])
            cube.scale.x = size
            cube.scale.y = size
            cube.scale.z = 0.01  # Dünne Fläche
            
            cube.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)  # Rot/transparent
            
            marker_array.markers.append(cube)
        
        self.hole_marker_pub.publish(marker_array)
    
    def label_connected_components(self, mask: np.ndarray) -> np.ndarray:
        """
        Findet zusammenhängende Komponenten in einer 2D-Maske (ohne scipy).
        
        Args:
            mask: Boolean-Maske (True = zur Komponente gehörend)
            
        Returns:
            Labeled Array mit IDs für jede Komponente
        """
        labeled = np.zeros_like(mask, dtype=int)
        current_label = 0
        
        for i in range(mask.shape[0]):
            for j in range(mask.shape[1]):
                if mask[i, j] and labeled[i, j] == 0:
                    # Starte neue Komponente
                    current_label += 1
                    self._fill_component(mask, labeled, i, j, current_label)
        
        return labeled
    
    def _fill_component(self, mask: np.ndarray, labeled: np.ndarray, i: int, j: int, label: int):
        """
        Füllt eine Komponente mit Flood Fill (rekursiv).
        """
        if i < 0 or i >= mask.shape[0] or j < 0 or j >= mask.shape[1]:
            return
        if not mask[i, j] or labeled[i, j] != 0:
            return
        
        labeled[i, j] = label
        
        # 4er-Nachbarschaft
        self._fill_component(mask, labeled, i + 1, j, label)
        self._fill_component(mask, labeled, i - 1, j, label)
        self._fill_component(mask, labeled, i, j + 1, label)
        self._fill_component(mask, labeled, i, j - 1, label)
    
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
            
            # Filter: Nur vertikale Flächen (auf dem Boden stehend)
            if not self.is_vertical_surface(normal, inlier_points):
                self.get_logger().debug(
                    f"Fläche {i+1} verworfen: nicht vertikal oder nicht am Boden"
                )
                remaining_points = remaining_points[~inliers]
                continue
            
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
