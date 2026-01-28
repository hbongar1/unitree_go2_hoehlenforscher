#!/usr/bin/env python3
"""
ROS2 Wall and Hole Detection Node

Verantwortlichkeit:
- Punktwolke vom Unitree LiDAR empfangen (/utlidar/cloud)
- Ebene (Wand) mit RANSAC erkennen
- Löcher (ca. 50x50cm) in der Wand detektieren
- Ergebnisse als Messages und Visualisierungsdaten publizieren

Anforderungen:
- ROS2 Humble
- Ubuntu 22.04+
- python3-open3d, numpy, scipy
"""

import rclpy
from rclpy.node import Node
import numpy as np
from typing import Tuple, Optional, List
import open3d as o3d

# ROS2 Messages
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray


class WallHoleDetectionNode(Node):
    """Wand- und Loch-Erkennungs Node"""
    
    def __init__(self):
        super().__init__('wall_hole_detection_node')
        
        # Subscriber für Lidar PointCloud
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/utlidar/cloud', self.cloud_callback, 10
        )
        
        # Publishers für Ergebnisse
        self.wall_pub = self.create_publisher(
            PointCloud2, '/detection/wall_plane', 10
        )
        self.hole_pub = self.create_publisher(
            PointCloud2, '/detection/hole_points', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/detection/wall_hole_markers', 10
        )
        
        # Parameters
        self.declare_parameter('distance_threshold', 0.05)  # 5cm Distanz zur Ebene
        self.declare_parameter('ransac_iterations', 1000)
        self.declare_parameter('min_plane_points', 100)
        self.declare_parameter('hole_width', 0.50)  # 50cm
        self.declare_parameter('hole_height', 0.50)  # 50cm
        self.declare_parameter('hole_min_points', 20)  # Min. Punkte für ein Loch
        
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.ransac_iterations = self.get_parameter('ransac_iterations').value
        self.min_plane_points = self.get_parameter('min_plane_points').value
        self.hole_width = self.get_parameter('hole_width').value
        self.hole_height = self.get_parameter('hole_height').value
        self.hole_min_points = self.get_parameter('hole_min_points').value
        
        self.get_logger().info('Wall and Hole Detection Node gestartet')
        
    def cloud_callback(self, msg: PointCloud2):
        """Callback für Lidar PointCloud"""
        try:
            # Konvertiere PointCloud2 zu numpy array
            points = self._pointcloud2_to_array(msg)
            
            if points is None or len(points) < self.min_plane_points:
                self.get_logger().warn('Nicht genug Punkte in der Cloud')
                return
            
            # Erkenne die Wand (Ebene)
            wall_points, plane_model = self._detect_wall_plane(points)
            
            if wall_points is None or len(wall_points) < self.min_plane_points:
                self.get_logger().warn('Keine Wand erkannt')
                return
            
            self.get_logger().info(f'Wand erkannt mit {len(wall_points)} Punkten')
            
            # Detektiere Löcher in der Wand
            holes = self._detect_holes(points, wall_points, plane_model)
            
            if holes:
                self.get_logger().info(f'{len(holes)} Löcher erkannt')
            
            # Publiziere Ergebnisse
            self._publish_results(msg.header, wall_points, holes, plane_model)
            
        except Exception as e:
            self.get_logger().error(f'Fehler bei der Verarbeitung: {str(e)}')
    
    def _pointcloud2_to_array(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """Konvertiere PointCloud2 zu numpy array"""
        try:
            points = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
            points_array = np.array(list(points))
            
            if len(points_array) == 0:
                return None
            
            return points_array
        except Exception as e:
            self.get_logger().error(f'Fehler bei PointCloud2 Konvertierung: {str(e)}')
            return None
    
    def _detect_wall_plane(self, points: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Erkenne die Wandebene mit RANSAC
        Gibt zurück: (wall_points, plane_model [a, b, c, d])
        Ebene: ax + by + cz + d = 0
        """
        try:
            # Erstelle Open3D PointCloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # Filtere Punkte außerhalb eines sinnvollen Bereichs
            # (z.B. maximal 5 Meter Entfernung)
            distances = np.linalg.norm(points, axis=1)
            mask = distances < 5.0
            pcd_filtered = pcd.select_by_index(np.where(mask)[0])
            
            if len(pcd_filtered.points) < self.min_plane_points:
                return None, None
            
            # RANSAC für Ebene
            plane_model, inliers = pcd_filtered.segment_plane(
                distance_threshold=self.distance_threshold,
                ransac_n=3,
                num_iterations=self.ransac_iterations
            )
            
            wall_points = np.asarray(pcd_filtered.points)[inliers]
            
            return wall_points, np.array(plane_model)
            
        except Exception as e:
            self.get_logger().error(f'Fehler bei Ebenen-Erkennung: {str(e)}')
            return None, None
    
    def _project_to_plane(self, points: np.ndarray, plane_model: np.ndarray) -> np.ndarray:
        """Projiziere Punkte auf die Ebene"""
        a, b, c, d = plane_model
        
        # Normalisiere die Ebenenkoeffizienten
        norm = np.sqrt(a**2 + b**2 + c**2)
        a, b, c = a / norm, b / norm, c / norm
        d = d / norm
        
        # Berechne den Projektionsfaktor für jeden Punkt
        distances = (a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d)
        
        # Projiziere Punkte
        projected = points - distances[:, np.newaxis] * np.array([a, b, c])
        
        return projected
    
    def _detect_holes(self, all_points: np.ndarray, wall_points: np.ndarray, 
                      plane_model: np.ndarray) -> List[dict]:
        """
        Detektiere Löcher in der Wandebene
        Strategie: Finde Bereiche in der Wand mit signifikant weniger Punkten
        """
        holes = []
        
        try:
            # Projiziere alle Punkte auf die Wand-Ebene
            projected_all = self._project_to_plane(all_points, plane_model)
            
            # Erstelle ein 2D-Grid in der Ebene
            # Bestimme die lokale Koordinaten der Wand
            wall_points_2d, _, _ = self._get_plane_coordinates(wall_points, plane_model)
            
            if len(wall_points_2d) < self.min_plane_points:
                return holes
            
            # Bestimme die Ausdehnung der Wand
            x_min, x_max = wall_points_2d[:, 0].min(), wall_points_2d[:, 0].max()
            y_min, y_max = wall_points_2d[:, 1].min(), wall_points_2d[:, 1].max()
            
            # Erstelle ein Gitter für die Suche nach Löchern
            grid_size = 0.1  # 10cm x 10cm Zellen
            x_bins = np.arange(x_min, x_max, grid_size)
            y_bins = np.arange(y_min, y_max, grid_size)
            
            # Projektion aller Punkte auf die Wand-Ebene als 2D
            all_points_2d, _, _ = self._get_plane_coordinates(projected_all, plane_model)
            
            # Erstelle ein Punktdichte-Grid
            hole_grid = {}
            for point_2d in all_points_2d:
                x_idx = int((point_2d[0] - x_min) / grid_size)
                y_idx = int((point_2d[1] - y_min) / grid_size)
                
                key = (x_idx, y_idx)
                hole_grid[key] = hole_grid.get(key, 0) + 1
            
            # Finde Zellen mit niedriger Punktdichte (potenzielle Löcher)
            # Berechne durchschnittliche Punktdichte
            if hole_grid:
                avg_density = np.mean(list(hole_grid.values()))
                density_threshold = avg_density * 0.2  # Löcher haben < 20% der durchschnittlichen Dichte
                
                # Finde zusammenhängende Gruppen von niedrigen Dichte-Zellen
                hole_candidates = []
                for (x_idx, y_idx), density in hole_grid.items():
                    if density < density_threshold:
                        x_pos = x_min + x_idx * grid_size
                        y_pos = y_min + y_idx * grid_size
                        hole_candidates.append((x_pos, y_pos, density))
                
                # Clustere benachbarte niedrige Dichte-Zellen
                holes = self._cluster_holes(hole_candidates, grid_size)
                
                # Filtere zu kleine Löcher
                holes = [h for h in holes if h['size'] >= self.hole_min_points]
            
            return holes
            
        except Exception as e:
            self.get_logger().error(f'Fehler bei Loch-Erkennung: {str(e)}')
            return holes
    
    def _get_plane_coordinates(self, points: np.ndarray, plane_model: np.ndarray) -> Tuple:
        """
        Konvertiere 3D-Punkte zu 2D-Ebenen-Koordinaten
        Gibt zurück: (2D_points, u_axis, v_axis)
        """
        a, b, c, d = plane_model
        
        # Normalisiere die Ebenenkoeffizienten
        norm = np.sqrt(a**2 + b**2 + c**2)
        normal = np.array([a / norm, b / norm, c / norm])
        
        # Finde zwei orthogonale Vektoren zur Normale
        if abs(normal[0]) < 0.9:
            u_axis = np.cross(normal, np.array([1, 0, 0]))
        else:
            u_axis = np.cross(normal, np.array([0, 1, 0]))
        
        u_axis = u_axis / np.linalg.norm(u_axis)
        v_axis = np.cross(normal, u_axis)
        v_axis = v_axis / np.linalg.norm(v_axis)
        
        # Projiziere Punkte auf die 2D-Ebene
        points_2d = np.column_stack([
            np.dot(points, u_axis),
            np.dot(points, v_axis)
        ])
        
        return points_2d, u_axis, v_axis
    
    def _cluster_holes(self, hole_candidates: List[Tuple], grid_size: float) -> List[dict]:
        """Clustere benachbarte Loch-Kandidaten"""
        if not hole_candidates:
            return []
        
        holes = []
        visited = set()
        
        def get_neighbors(x_idx, y_idx):
            """Finde benachbarte Zellen"""
            neighbors = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    neighbors.append((x_idx + dx, y_idx + dy))
            return neighbors
        
        for i, (x, y, density) in enumerate(hole_candidates):
            if i in visited:
                continue
            
            # BFS für Clustering
            cluster = [(i, x, y)]
            visited.add(i)
            queue = [(i, x, y)]
            
            while queue:
                idx, cx, cy = queue.pop(0)
                x_idx = int((cx - (hole_candidates[0][0] - grid_size * 5)) / grid_size)
                y_idx = int((cy - (hole_candidates[0][1] - grid_size * 5)) / grid_size)
                
                for neighbor_idx, (nx, ny, _) in enumerate(hole_candidates):
                    if neighbor_idx not in visited:
                        # Prüfe ob benachbart (innerhalb von 1.5x grid_size)
                        if abs(nx - cx) < grid_size * 1.5 and abs(ny - cy) < grid_size * 1.5:
                            visited.add(neighbor_idx)
                            cluster.append((neighbor_idx, nx, ny))
                            queue.append((neighbor_idx, nx, ny))
            
            # Berechne Cluster-Eigenschaften
            if len(cluster) >= 2:  # Mindestens 2 Zellen für ein Loch
                center_x = np.mean([c[1] for c in cluster])
                center_y = np.mean([c[2] for c in cluster])
                size = len(cluster)
                
                holes.append({
                    'center_x': center_x,
                    'center_y': center_y,
                    'size': size,
                    'diameter': np.sqrt(size) * grid_size
                })
        
        return holes
    
    def _publish_results(self, header: Header, wall_points: np.ndarray, 
                        holes: List[dict], plane_model: np.ndarray):
        """Publiziere Erkennungsergebnisse"""
        
        # Publiziere Wandpunkte
        wall_pc = self._array_to_pointcloud2(wall_points, header)
        self.wall_pub.publish(wall_pc)
        
        # Publiziere Marker
        markers = MarkerArray()
        
        # Marker für die Wand-Ebene
        wall_marker = Marker()
        wall_marker.header = header
        wall_marker.id = 0
        wall_marker.type = Marker.CUBE
        wall_marker.action = Marker.ADD
        wall_marker.pose.position.x = 0.0
        wall_marker.pose.position.y = 0.0
        wall_marker.pose.position.z = 0.0
        wall_marker.scale.x = 10.0
        wall_marker.scale.y = 10.0
        wall_marker.scale.z = 0.1
        wall_marker.color.a = 0.3
        wall_marker.color.r = 0.5
        wall_marker.color.g = 0.5
        wall_marker.color.b = 1.0
        markers.markers.append(wall_marker)
        
        # Marker für Löcher
        for i, hole in enumerate(holes):
            hole_marker = Marker()
            hole_marker.header = header
            hole_marker.id = i + 1
            hole_marker.type = Marker.CUBE
            hole_marker.action = Marker.ADD
            hole_marker.pose.position.x = hole['center_x']
            hole_marker.pose.position.y = hole['center_y']
            hole_marker.pose.position.z = 0.0
            hole_marker.scale.x = self.hole_width
            hole_marker.scale.y = self.hole_height
            hole_marker.scale.z = 0.05
            hole_marker.color.a = 0.8
            hole_marker.color.r = 1.0
            hole_marker.color.g = 0.0
            hole_marker.color.b = 0.0
            markers.markers.append(hole_marker)
            
            self.get_logger().info(f'Loch {i+1}: Position ({hole["center_x"]:.2f}, {hole["center_y"]:.2f}), '
                                 f'Größe: {hole["diameter"]:.2f}m')
        
        self.marker_pub.publish(markers)
    
    def _array_to_pointcloud2(self, points: np.ndarray, header: Header) -> PointCloud2:
        """Konvertiere numpy array zu PointCloud2"""
        points_list = [tuple(p) for p in points]
        return pc2.create_cloud_xyz32(header, points_list)


def main(args=None):
    rclpy.init(args=args)
    node = WallHoleDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node wird beendet...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
