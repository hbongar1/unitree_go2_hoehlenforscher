"""
Loch-Erkennung mittels 2D-Projektion (Optimiert & Bereinigt)
Erkennt vertikale Löcher in Wänden durch Y-Z-Frontalprojektion.
"""

import rclpy
from typing import List
import numpy as np
import struct
import csv
import os
from datetime import datetime
from collections import deque
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from go2_my_nodes_py.base_node import BaseNode
from sensor_msgs.msg import PointCloud2, PointField
from go2_msgs.msg import Entrance
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Scipy imports (direkt, ohne NumPy Fallbacks)
from scipy.ndimage import gaussian_filter, binary_erosion, binary_dilation, label


class HoleDetectionCleanNode(BaseNode):
    """
    Erkennt Eingänge durch 2D-Projektion auf Y-Z-Ebene.
    
    Workflow:
    1. Empfange PointCloud2
    2. Filtere Punkte (100° Radius, 0.3-2.5m Tiefe)
    3. Projiziere auf Y-Z-Ebene (Frontalansicht)
    4. Finde Regionen mit niedriger Punkt-Dichte
    5. Validiere: Größe, 3-Seiten-Check
    6. Publiziere: ROS Topics, RViz Marker, CSV
    """
    
    def __init__(self):
        super().__init__('hole_detection_clean')
        
        # === PARAMETER ===
        
        # Löcher Spezifikationen
        self.min_hole_diameter = 0.08  # 8cm minimal
        self.max_hole_diameter = 2.0   # 2m maximal
        self.height_threshold = 0.1    # Min 10cm Höhe
        self.width_threshold = 0.1     # Min 10cm Breite
        
        # Voxel Grid (2D-Projektion)
        self.voxel_size = 0.02  # 2cm Auflösung
        self.gaussian_sigma = 1.5  # Glättung
        self.min_region_cells = 80  # Weniger streng für kleine Löcher
        self.min_surrounded_sides = 1  # Loch muss von mind. 1 Seite umgeben sein
        
        # Multi-Frame Tracking
        self.frame_buffer_size = 50
        self.point_buffer = deque(maxlen=self.frame_buffer_size)
        self.frame_counter = 0
        self.process_every_n_frames = 2
        
        # Konfidenz
        self.entrance_history = {}
        self.confidence_threshold = 1
        self.entrance_timeout = 40
        self.next_entrance_id = 0
        
        # CSV Logging
        log_dir = os.path.join(os.path.dirname(__file__), 'entrance_detections')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = os.path.join(log_dir, f'detections_{timestamp}.csv')
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Position_X_m', 'Position_Y_m', 'Position_Z_m',
                           'Breite_m', 'Hoehe_m', 'Tiefe_m', 'Confidence'])
        self.get_logger().info(f"CSV-Logging: {self.csv_file}")
        
        # === ROS2 SETUP ===
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.cloud_sub = self.create_subscription(
            PointCloud2, 'utlidar/cloud', self.cloud_data_callback, qos_profile
        )
        
        # Publishers
        self.entrance_pub = self.create_publisher(Entrance, '/entrance_detection', 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, '/filtered_cloud_voxel_grid', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/entrance_markers', 10)
        
        self.get_logger().info("✅ Hole Detection Clean Node gestartet!")
    
    # ====================================================================
    # MAIN CALLBACK
    # ====================================================================
    
    def cloud_data_callback(self, msg: PointCloud2):
        """Hauptlogik: PointCloud → Löcher finden → Publizieren"""
        try:
            # 1. Punkte extrahieren
            points = self.extract_points_from_cloud(msg)
            if len(points) == 0:
                return
            
            # 2. Filtern (100° Radius, 0.3-2.5m Tiefe)
            filtered_points = self.filter_points(points)
            if len(filtered_points) < 50:
                return
            
            # 3. Gefilterte Cloud publizieren (für RViz)
            self.publish_filtered_cloud(filtered_points, msg.header)
            
            # 4. Multi-Frame Akkumulation
            self.point_buffer.append(filtered_points)
            self.frame_counter += 1
            
            if self.frame_counter % self.process_every_n_frames != 0:
                return
            
            if len(self.point_buffer) < 2:
                return
            
            # 5. Kombiniere Frames
            all_points = np.vstack(list(self.point_buffer))
            
            # 6. Finde Loch-Regionen
            hole_regions = self.find_hole_regions_in_cloud(all_points)
            
            # 7. Analysiere & erstelle Entrance-Messages
            current_entrances = []
            for region in hole_regions:
                entrance = self.analyze_hole_region(region, msg.header)
                if entrance:
                    current_entrances.append(entrance)
            
            # 8. Update Tracking
            self.update_entrance_tracking(current_entrances)
            
            # 9. Publiziere stabile Eingänge
            self.publish_stable_entrances(msg.header)
            
        except Exception as e:
            self.get_logger().error(f"Fehler in callback: {e}")
    
    # ====================================================================
    # PUNKT EXTRAKTION & FILTERUNG
    # ====================================================================
    
    def extract_points_from_cloud(self, cloud: PointCloud2) -> np.ndarray:
        """Extrahiert XYZ-Punkte aus PointCloud2"""
        points_list = []
        point_step = cloud.point_step
        row_step = cloud.row_step
        data = cloud.data
        
        for v in range(cloud.height):
            for u in range(cloud.width):
                offset = v * row_step + u * point_step
                x, y, z = struct.unpack_from('fff', data, offset)
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    points_list.append([x, y, z])
        
        return np.array(points_list) if points_list else np.array([]).reshape(0, 3)
    
    def filter_points(self, points: np.ndarray) -> np.ndarray:
        """
        Filtert Punkte:
        - 100° Radius (±50°)
        - 0.3-2.5m Tiefe (mit Randabstand)
        - Höhe: 0-3m
        """
        if len(points) == 0:
            return points

        # Polar-Koordinaten
        angles = np.arctan2(points[:, 1], points[:, 0]) * 180.0 / np.pi
        distances_xy = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        
        # Filter 1: Winkel (±45°, mit 5° Randabstand)
        angle_filter = np.abs(angles) < 45.0
        
        # Filter 2: Distanz (0.3-2.5m mit Randabstand)
        distance_filter = (distances_xy > 0.4) & (distances_xy < 2.4)
        
        # Filter 3: Höhe (0-3m)
        z_filter = (points[:, 2] > 0.0) & (points[:, 2] < 3.0)
        
        # Kombiniere
        final_filter = angle_filter & distance_filter & z_filter
        return points[final_filter]
    
    # ====================================================================
    # LOCH-ERKENNUNG (KERN-ALGORITHMUS)
    # ====================================================================
    
    def find_hole_regions_in_cloud(self, points: np.ndarray) -> List[dict]:
        """
        Findet vertikale Löcher durch Y-Z-Frontalprojektion.
        
        Returns:
            Liste von Loch-Regionen mit:
            - centroid: [x, y, z]
            - bounds_min/max: Bounding Box
            - width/height: Dimensionen
            - depth: Tiefe in X-Richtung
        """
        if len(points) < 50:
            return []
        
        # Filtere Punkte vor dem Roboter (0.3-2.5m)
        x_min_filter, x_max_filter = 0.3, 2.5
        in_front = (points[:, 0] >= x_min_filter) & (points[:, 0] <= x_max_filter)
        front_points = points[in_front]
        
        if len(front_points) < 50:
            return []
        
        # Y-Z-Projektion (Frontalansicht)
        y_points = front_points[:, 1]
        z_points = front_points[:, 2]
        
        y_min, y_max = np.min(y_points), np.max(y_points)
        z_min, z_max = np.min(z_points), np.max(z_points)
        
        # Erstelle 2D-Dichte-Grid
        grid_resolution_2d = self.voxel_size
        y_bins = int((y_max - y_min) / grid_resolution_2d) + 1
        z_bins = int((z_max - z_min) / grid_resolution_2d) + 1
        
        # Performance-Limit
        if y_bins > 1000 or z_bins > 1000:
            grid_resolution_2d = 0.02
            y_bins = int((y_max - y_min) / grid_resolution_2d) + 1
            z_bins = int((z_max - z_min) / grid_resolution_2d) + 1
        
        # Z ähle Punkte in Grid
        density_2d = np.zeros((y_bins, z_bins), dtype=np.float32)
        y_indices = ((y_points - y_min) / grid_resolution_2d).astype(int)
        z_indices = ((z_points - z_min) / grid_resolution_2d).astype(int)
        
        for yi, zi in zip(y_indices, z_indices):
            if 0 <= yi < y_bins and 0 <= zi < z_bins:
                density_2d[yi, zi] += 1
        
        # Glättung
        smoothed = gaussian_filter(density_2d, sigma=self.gaussian_sigma)
        
        # Finde niedrige Dichte (Löcher)
        if not np.any(smoothed > 0):
            return []
        
        threshold = np.percentile(smoothed[smoothed > 0], 15)
        hole_mask = smoothed < threshold
        
        # Morphologie (weniger Aufblähung für genauere Maße)
        hole_mask = binary_dilation(hole_mask, iterations=1)
        
        # Label zusammenhängende Regionen
        labeled_array, num_features = label(hole_mask)
        
        # Analysiere jede Region
        hole_regions = []
        for region_id in range(1, num_features + 1):
            region_cells = np.argwhere(labeled_array == region_id)
            
            # Mindestgröße
            if len(region_cells) < self.min_region_cells:
                continue
            
            # Weltkoordinaten
            region_y_min = region_cells[:, 0].min() * grid_resolution_2d + y_min
            region_y_max = region_cells[:, 0].max() * grid_resolution_2d + y_min
            region_z_min = region_cells[:, 1].min() * grid_resolution_2d + z_min
            region_z_max = region_cells[:, 1].max() * grid_resolution_2d + z_min
            
            # Dimensionen mit Voxel-Korrektur (+1 statt +2)
            width_voxels = region_cells[:, 0].max() - region_cells[:, 0].min() + 1
            height_voxels = region_cells[:, 1].max() - region_cells[:, 1].min() + 1
            
            width = (width_voxels + 1) * grid_resolution_2d  # +1 Voxel Rand-Korrektur
            height = (height_voxels + 1) * grid_resolution_2d
            
            # Größen-Check
            if width < self.min_hole_diameter or width > self.max_hole_diameter:
                continue
            if height < self.height_threshold:
                continue
            
            # 3-Seiten-Check (Rand-Filter)
            margin = grid_resolution_2d * 5
            left = np.any((front_points[:, 1] < (region_y_min - margin)) &
                         (front_points[:, 2] >= region_z_min) &
                         (front_points[:, 2] <= region_z_max))
            right = np.any((front_points[:, 1] > (region_y_max + margin)) &
                          (front_points[:, 2] >= region_z_min) &
                          (front_points[:, 2] <= region_z_max))
            top = np.any((front_points[:, 2] > (region_z_max + margin)) &
                        (front_points[:, 1] >= region_y_min) &
                        (front_points[:, 1] <= region_y_max))
            bottom = np.any((front_points[:, 2] < (region_z_min - margin)) &
                           (front_points[:, 1] >= region_y_min) &
                           (front_points[:, 1] <= region_y_max))
            
            surrounded_sides = sum([left, right, top, bottom])
            if surrounded_sides < self.min_surrounded_sides:
                continue
            
            # X-Dimension (Tiefe)
            in_region_y = (front_points[:, 1] >= region_y_min) & (front_points[:, 1] <= region_y_max)
            in_region_z = (front_points[:, 2] >= region_z_min) & (front_points[:, 2] <= region_z_max)
            in_region = in_region_y & in_region_z
            
            if not np.any(in_region):
                continue
            
            region_points = front_points[in_region]
            x_min = np.min(region_points[:, 0])
            x_max = np.max(region_points[:, 0])
            depth = x_max - x_min
            
            # Zentroid
            y_center_voxel = (region_cells[:, 0].min() + region_cells[:, 0].max()) / 2.0
            z_center_voxel = (region_cells[:, 1].min() + region_cells[:, 1].max()) / 2.0
            y_center = y_center_voxel * grid_resolution_2d + y_min
            z_center = z_center_voxel * grid_resolution_2d + z_min
            
            centroid_x = (x_min + x_max) / 2.0
            
            hole_regions.append({
                'centroid': np.array([centroid_x, y_center, z_center]),
                'bounds_min': np.array([x_min, region_y_min, region_z_min]),
                'bounds_max': np.array([x_max, region_y_max, region_z_max]),
                'width': width,
                'height': height,
                'depth': depth,
                'voxel_count': len(region_cells)
            })
        
        return hole_regions
    
    # ====================================================================
    # ANALYSE & TRACKING
    # ====================================================================
    
    def analyze_hole_region(self, region: dict, header: Header) -> Entrance:
        """Erstellt Entrance-Message aus Loch-Region"""
        entrance_msg = Entrance()
        entrance_msg.header = header
        
        pos = Point()
        pos.x = float(region['centroid'][0])
        pos.y = float(region['centroid'][1])
        pos.z = float(region['centroid'][2])
        entrance_msg.position = pos
        
        entrance_msg.width = float(region['width'])
        entrance_msg.height = float(region['height'])
        entrance_msg.depth = float(region.get('depth', 0.0))
        entrance_msg.voxel_count = int(region.get('voxel_count', 0))
        
        return entrance_msg
    
    def update_entrance_tracking(self, current_entrances: List[Entrance]):
        """Update Konfidenz-Tracking mit Measurement-Smoothing"""
        smoothing_alpha = 0.2  # 20% neue Messung, 80% alte
        
        # Inkrementiere frames_since_seen
        for data in self.entrance_history.values():
            data['frames_since_seen'] += 1
        
        # Lösche alte Eingänge
        to_delete = [eid for eid, data in self.entrance_history.items()
                    if data['frames_since_seen'] > self.entrance_timeout]
        for eid in to_delete:
            del self.entrance_history[eid]
        
        # Match neue Eingänge
        for entrance in current_entrances:
            pos = entrance.position
            matched_id = None
            
            # Suche nächsten bekannten Eingang
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
                # Update bekannter Eingang
                self.entrance_history[matched_id]['confidence'] += 1
                self.entrance_history[matched_id]['frames_since_seen'] = 0
                self.entrance_history[matched_id]['entrance'] = entrance
                
                # Glätte Maße (Exponential Moving Average)
                prev_width = self.entrance_history[matched_id].get('smooth_width', entrance.width)
                prev_height = self.entrance_history[matched_id].get('smooth_height', entrance.height)
                prev_depth = self.entrance_history[matched_id].get('smooth_depth', entrance.depth)
                
                self.entrance_history[matched_id]['smooth_width'] = \
                    (1 - smoothing_alpha) * prev_width + smoothing_alpha * entrance.width
                self.entrance_history[matched_id]['smooth_height'] = \
                    (1 - smoothing_alpha) * prev_height + smoothing_alpha * entrance.height
                self.entrance_history[matched_id]['smooth_depth'] = \
                    (1 - smoothing_alpha) * prev_depth + smoothing_alpha * entrance.depth
            else:
                # Neuer Eingang
                new_id = self.next_entrance_id
                self.next_entrance_id += 1
                self.entrance_history[new_id] = {
                    'position': pos,
                    'entrance': entrance,
                    'confidence': 1,
                    'frames_since_seen': 0,
                    'smooth_width': entrance.width,
                    'smooth_height': entrance.height,
                    'smooth_depth': entrance.depth
                }
    
    # ====================================================================
    # PUBLISHING
    # ====================================================================
    
    def publish_stable_entrances(self, header: Header):
        """Publiziert stabile Eingänge (mit CSV-Logging)"""
        published_count = 0
        marker_array = MarkerArray()
        
        for entrance_id, data in self.entrance_history.items():
            if data['confidence'] >= self.confidence_threshold:
                entrance = data['entrance']
                
                # Nutze geglättete Maße
                smooth_width = data.get('smooth_width', entrance.width)
                smooth_height = data.get('smooth_height', entrance.height)
                smooth_depth = data.get('smooth_depth', entrance.depth)
                
                entrance.header = header
                entrance.width = float(smooth_width)
                entrance.height = float(smooth_height)
                
                # ROS Publish
                self.entrance_pub.publish(entrance)
                published_count += 1
                
                # CSV Logging
                try:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    with open(self.csv_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            timestamp,
                            f'{entrance.position.x:.4f}',
                            f'{entrance.position.y:.4f}',
                            f'{entrance.position.z:.4f}',
                            f'{smooth_width:.4f}',
                            f'{smooth_height:.4f}',
                            f'{smooth_depth:.4f}',
                            data['confidence']
                        ])
                except Exception as e:
                    self.get_logger().warn(f"CSV-Logging fehlgeschlagen: {e}")
                
                # RViz Marker
                marker = Marker()
                marker.header = header
                marker.ns = "entrances"
                marker.id = entrance_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                marker.pose.position = entrance.position
                # 180° Z-Rotation für X-Z-Ebene
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 1.0
                marker.pose.orientation.w = 0.0
                
                marker.scale.x = 0.1  # Dünn (Marker-Dicke)
                marker.scale.y = smooth_width
                marker.scale.z = smooth_height
                
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.7
                
                marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
                marker_array.markers.append(marker)
        
        if published_count > 0:
            self.marker_pub.publish(marker_array)
            self.get_logger().info(f"{published_count} Eingänge → CSV: {self.csv_file}")
    
    def publish_filtered_cloud(self, points: np.ndarray, header: Header):
        """Publiziert gefilterte PointCloud für RViz"""
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


def main(args=None):
    rclpy.init(args=args)
    node = HoleDetectionCleanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
