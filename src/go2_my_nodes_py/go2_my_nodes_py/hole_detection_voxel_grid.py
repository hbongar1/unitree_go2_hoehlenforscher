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


# ============================================================================
# NumPy-basierte Fallback-Funktionen (falls scipy nicht verfügbar)
# ============================================================================

def numpy_gaussian_filter(input_array: np.ndarray, sigma: float) -> np.ndarray:
    """
    Einfacher Gaussian Filter mit NumPy (Fallback für scipy.ndimage.gaussian_filter)
    Funktioniert für 2D und 3D Arrays
    """
    if input_array.ndim == 2:
        # 2D Gaussian Filter
        kernel_size = int(4 * sigma + 0.5)
        if kernel_size % 2 == 0:
            kernel_size += 1
        
        # Erstelle 1D Gaussian Kernel
        x = np.arange(kernel_size) - kernel_size // 2
        gaussian_1d = np.exp(-x**2 / (2 * sigma**2))
        gaussian_1d /= gaussian_1d.sum()
        
        # 2D Kernel durch Outer Product
        kernel_2d = np.outer(gaussian_1d, gaussian_1d)
        
        # Konvolution mit Padding
        padded = np.pad(input_array, kernel_size // 2, mode='edge')
        result = np.zeros_like(input_array, dtype=np.float32)
        
        for i in range(input_array.shape[0]):
            for j in range(input_array.shape[1]):
                window = padded[i:i+kernel_size, j:j+kernel_size]
                result[i, j] = np.sum(window * kernel_2d)
        
        return result
    
    elif input_array.ndim == 3:
        # 3D Gaussian Filter (vereinfacht - wendet 1D Filter in jeder Richtung an)
        kernel_size = int(4 * sigma + 0.5)
        if kernel_size % 2 == 0:
            kernel_size += 1
        
        # 1D Gaussian Kernel
        x = np.arange(kernel_size) - kernel_size // 2
        gaussian_1d = np.exp(-x**2 / (2 * sigma**2))
        gaussian_1d /= gaussian_1d.sum()
        
        # Sequentielle Filterung entlang jeder Achse
        result = input_array.astype(float).copy()
        
        # X-Achse
        padded = np.pad(result, ((kernel_size//2, kernel_size//2), (0, 0), (0, 0)), mode='edge')
        temp = np.zeros_like(result)
        for i in range(result.shape[0]):
            for j in range(result.shape[1]):
                for k in range(result.shape[2]):
                    temp[i, j, k] = np.sum(padded[i:i+kernel_size, j, k] * gaussian_1d)
        result = temp
        
        return result.astype(np.float32)
    
    else:
        # Fallback: keine Glättung
        return input_array.astype(np.float32)


def numpy_binary_erosion(input_array: np.ndarray, iterations: int = 1) -> np.ndarray:
    """
    Einfache binäre Erosion mit NumPy (Fallback für scipy.ndimage.binary_erosion)
    """
    result = input_array.copy()
    
    for _ in range(iterations):
        eroded = result.copy()
        rows, cols = result.shape
        
        for i in range(1, rows - 1):
            for j in range(1, cols - 1):
                # 3x3 Nachbarschaft prüfen
                if result[i, j]:
                    # Wenn irgendeiner der 8 Nachbarn False ist, erodiere
                    neighbors = [
                        result[i-1, j-1], result[i-1, j], result[i-1, j+1],
                        result[i, j-1],                    result[i, j+1],
                        result[i+1, j-1], result[i+1, j], result[i+1, j+1]
                    ]
                    if not all(neighbors):
                        eroded[i, j] = False
        
        result = eroded
    
    return result


def numpy_binary_dilation(input_array: np.ndarray, iterations: int = 1) -> np.ndarray:
    """
    Einfache binäre Dilation mit NumPy (Fallback für scipy.ndimage.binary_dilation)
    """
    result = input_array.copy()
    
    for _ in range(iterations):
        dilated = result.copy()
        rows, cols = result.shape
        
        for i in range(1, rows - 1):
            for j in range(1, cols - 1):
                # 3x3 Nachbarschaft prüfen
                if not result[i, j]:
                    # Wenn irgendeiner der 8 Nachbarn True ist, expandiere
                    neighbors = [
                        result[i-1, j-1], result[i-1, j], result[i-1, j+1],
                        result[i, j-1],                    result[i, j+1],
                        result[i+1, j-1], result[i+1, j], result[i+1, j+1]
                    ]
                    if any(neighbors):
                        dilated[i, j] = True
        
        result = dilated
    
    return result


def numpy_label(input_array: np.ndarray) -> tuple:
    """
    Einfaches Connected Component Labeling mit NumPy (Fallback für scipy.ndimage.label)
    Verwendet Flood-Fill Algorithmus
    """
    labeled = np.zeros_like(input_array, dtype=np.int32)
    current_label = 0
    rows, cols = input_array.shape
    
    def flood_fill(i, j, label):
        """Rekursiver Flood-Fill"""
        if i < 0 or i >= rows or j < 0 or j >= cols:
            return
        if not input_array[i, j] or labeled[i, j] != 0:
            return
        
        labeled[i, j] = label
        
        # 4-Connectivity (oben, unten, links, rechts)
        flood_fill(i-1, j, label)
        flood_fill(i+1, j, label)
        flood_fill(i, j-1, label)
        flood_fill(i, j+1, label)
    
    for i in range(rows):
        for j in range(cols):
            if input_array[i, j] and labeled[i, j] == 0:
                current_label += 1
                flood_fill(i, j, current_label)
    
    return labeled, current_label


# ============================================================================
# Hauptklasse
# ============================================================================

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
        self.region_marker_pub = self.create_publisher(MarkerArray, 'hole_region_markers_voxel_grid', 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, 'filtered_cloud_voxel_grid', 10)
        
        # Parameter aus aktuellen Thresholds
        self.height_threshold = 0.2
        self.width_threshold = 0.2
        self.max_width = 1.0
        self.cluster_distance = 0.5
        
        # Voxel Grid spezifische Parameter - optimiert für 20cm, 42cm, 50cm Löcher
        self.voxel_size = 0.01  # 1cm Voxel = guter Kompromiss (20cm Loch = 20x20 Voxel)
        self.gradient_percentile = 75  # Top 25% Gradienten = Kanten
        self.low_density_percentile = 25  # Unten 25% Dichte = Loch-Kandidaten
        self.gaussian_sigma = 0.8  # Reduzierte Glättung für kleine Löcher
        
        # Spezifische Parameter für bodenstehende Löcher
        self.min_hole_diameter = 0.15  # Minimaler Durchmesser 15cm (mit Sicherheitsmarge für 20cm)
        self.max_hole_diameter = 2.0  # Maximaler Durchmesser 2m
        self.vertical_slice_height = 0.08  # 8cm Scheiben (Kompromiss für alle Größen)
        self.min_region_cells = 15  # Mindestanzahl Grid-Zellen für gültige Region
        
        # Ground Plane Detection Parameter
        self.enable_ground_detection = True
        self.ground_plane_distance_threshold = 0.02  # 2cm tolerance for RANSAC
        self.ground_plane_min_points = 100  # Min points to fit plane
        self.min_height_above_ground = 0.15  # Ignore points < 15cm above ground
        
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
            f"Voxel Grid Node initialisiert (optimiert für 20/42/50cm Löcher):\n"
            f"  - Voxel-Größe: {self.voxel_size*100:.1f}cm\n"
            f"  - Min/Max Loch-Durchmesser: {self.min_hole_diameter*100:.0f}-{self.max_hole_diameter*100:.0f}cm\n"
            f"  - Scheiben-Höhe: {self.vertical_slice_height*100:.0f}cm\n"
            f"  - Glättungs-Sigma: {self.gaussian_sigma}\n"
            f"  - Bodenerkennung: {'Aktiviert' if self.enable_ground_detection else 'Deaktiviert'}"
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
            
            # Ground plane detection
            if self.enable_ground_detection:
                ground_result = self.detect_ground_plane(filtered_points)
                if ground_result is not None:
                    plane_model, ground_height = ground_result
                    above_ground, ground = self.filter_ground_points(
                        filtered_points, plane_model
                    )
                    filtered_points = above_ground
                    self.get_logger().debug(
                        f"Boden erkannt bei {ground_height:.2f}m, "
                        f"{len(above_ground)} Punkte über dem Boden"
                    )
            
            self.point_buffer.append(filtered_points)
            
            if self.frame_counter % self.process_every_n_frames != 0:
                return
            
            if len(self.point_buffer) < 3:
                self.get_logger().info("Warte auf mehr Frames...")
                return
            
            combined_points = np.vstack(list(self.point_buffer))
            
            self.publish_filtered_cloud(combined_points, msg.header)
            
            # Analysiere gesamte Point Cloud direkt
            hole_regions = self.find_hole_regions_in_cloud(combined_points)
            self.get_logger().info(f"[Voxel Grid] Gefundene Loch-Regionen: {len(hole_regions)}")
            
            self.publish_region_markers(hole_regions, msg.header)
            
            current_entrances = []
            for i, region in enumerate(hole_regions):
                entrance = self.analyze_hole_region(region, msg.header)
                if entrance:
                    current_entrances.append(entrance)
                    self.get_logger().debug(
                        f"Region {i}: Eingangs-Kandidat bei "
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
    
    def detect_ground_plane(self, points: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
        """
        Detects ground plane using RANSAC.
        
        Returns:
            (plane_model, ground_height): Plane coefficients [a,b,c,d] and avg height
            None if detection fails
        """
        if len(points) < self.ground_plane_min_points:
            return None
        
        try:
            from sklearn.linear_model import RANSACRegressor
            
            # Use bottom 50% of points (likely ground)
            z_sorted_indices = np.argsort(points[:, 2])
            bottom_half_count = min(len(points) // 2, 1000)  # Max 1000 points for speed
            bottom_indices = z_sorted_indices[:bottom_half_count]
            bottom_points = points[bottom_indices]
            
            if len(bottom_points) < self.ground_plane_min_points:
                return None
            
            # RANSAC: Fit plane ax + by + c = z
            X = bottom_points[:, :2]  # x, y
            y = bottom_points[:, 2]   # z
            
            ransac = RANSACRegressor(
                residual_threshold=self.ground_plane_distance_threshold,
                min_samples=50,
                max_trials=100,
                random_state=42
            )
            ransac.fit(X, y)
            
            # Plane model: z = ax + by + c  →  ax + by - z + c = 0
            a, b = ransac.estimator_.coef_
            c = ransac.estimator_.intercept_
            
            # Average ground height (z when x=0, y=0)
            ground_height = float(c)
            
            # Store as [a, b, -1, c] for easier distance calculation
            plane_model = np.array([a, b, -1.0, c])
            
            return plane_model, ground_height
        
        except ImportError:
            self.get_logger().warn("sklearn nicht verfügbar, Bodenerkennung übersprungen")
            return None
        except Exception as e:
            self.get_logger().warn(f"Bodenerkennung fehlgeschlagen: {e}")
            return None
    
    def filter_ground_points(
        self, points: np.ndarray, plane_model: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Separates ground points from above-ground points.
        
        Args:
            points: All points
            plane_model: [a, b, c, d] from plane equation ax + by + cz + d = 0
        
        Returns:
            (above_ground_points, ground_points)
        """
        # Calculate distance to plane: |ax + by + cz + d| / sqrt(a² + b² + c²)
        a, b, c, d = plane_model
        numerator = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d)
        denominator = np.sqrt(a**2 + b**2 + c**2)
        distances = numerator / denominator
        
        # Points within threshold are ground
        is_ground = distances < self.ground_plane_distance_threshold
        
        # Additionally: Points must be at least min_height_above_ground above the plane
        # Calculate signed distance (positive = above plane)
        signed_distances = (a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d) / denominator
        is_above_threshold = signed_distances > self.min_height_above_ground
        
        # Above ground = not ground AND above height threshold
        is_above_ground = (~is_ground) & is_above_threshold
        
        above_ground_points = points[is_above_ground]
        ground_points = points[~is_above_ground]
        
        return above_ground_points, ground_points
    
    def find_hole_regions_in_cloud(self, points: np.ndarray) -> List[dict]:
        """
        Findet bodenstehende Löcher durch 2D-Projektion und vertikale Analyse.
        Speziell für Löcher, die nach unten offen sind und auf dem Boden stehen.
        
        Returns:
            Liste von Loch-Regionen mit Zentroid, Bounding Box und Punkten
        """
        # Importiere scipy-Funktionen oder Fallback zu NumPy
        try:
            from scipy.ndimage import gaussian_filter, label, binary_erosion, binary_dilation
            use_scipy = True
            self.get_logger().debug("Verwende scipy für Bildverarbeitung")
        except ImportError:
            # Fallback zu NumPy-Implementierungen
            gaussian_filter = numpy_gaussian_filter
            label = numpy_label
            binary_erosion = numpy_binary_erosion
            binary_dilation = numpy_binary_dilation
            use_scipy = False
            self.get_logger().info("scipy nicht verfügbar - verwende NumPy-Fallback")
        
        if len(points) < 50:
            return []
        
        # Finde den niedrigsten Z-Wert (Boden/Basis)
        min_z = np.min(points[:, 2])
        max_z = np.max(points[:, 2])
        
        # Analysiere mehrere horizontale Scheiben von unten nach oben
        slice_heights = np.arange(
            min_z + 0.05,  # Beginne 5cm über dem Boden
            max_z,
            self.vertical_slice_height
        )
        
        if len(slice_heights) < 2:
            self.get_logger().warn("Nicht genug vertikale Scheiben für Analyse")
            return []
        
        # Erstelle 2D-Projektionen für jede Scheibe
        slice_masks = []
        xy_bounds = [np.min(points[:, 0]), np.max(points[:, 0]),
                     np.min(points[:, 1]), np.max(points[:, 1])]
        
        grid_resolution_2d = self.voxel_size  # Gleiche Auflösung wie Voxel
        
        for height in slice_heights:
            # Selektiere Punkte in dieser Scheibe (±2.5cm)
            slice_mask = np.abs(points[:, 2] - height) < (self.vertical_slice_height / 2)
            slice_points = points[slice_mask]
            
            if len(slice_points) < 10:
                slice_masks.append(None)
                continue
            
            # Erstelle 2D-Dichte-Gitter (XY-Projektion)
            x_bins = int((xy_bounds[1] - xy_bounds[0]) / grid_resolution_2d) + 1
            y_bins = int((xy_bounds[3] - xy_bounds[2]) / grid_resolution_2d) + 1
            
            density_2d = np.zeros((x_bins, y_bins), dtype=np.float32)
            
            # Zähle Punkte in jedem 2D-Grid-Zelle
            x_indices = ((slice_points[:, 0] - xy_bounds[0]) / grid_resolution_2d).astype(int)
            y_indices = ((slice_points[:, 1] - xy_bounds[2]) / grid_resolution_2d).astype(int)
            
            for xi, yi in zip(x_indices, y_indices):
                if 0 <= xi < x_bins and 0 <= yi < y_bins:
                    density_2d[xi, yi] += 1
            
            slice_masks.append(density_2d)
        
        # Finde konsistente Lücken über mehrere Scheiben hinweg
        # Eine Lücke = Bereich mit niedriger Dichte in mehreren aufeinanderfolgenden Scheiben
        valid_masks = [m for m in slice_masks if m is not None]
        
        if len(valid_masks) < 2:
            self.get_logger().warn("Nicht genug gültige Scheiben für Loch-Erkennung")
            return []
        
        # Kombiniere mehrere Scheiben durch Mittelwertbildung
        combined_density = np.mean(valid_masks, axis=0)
        
        # Adaptive Glättung basierend auf Voxel-Größe
        smoothed = gaussian_filter(combined_density, sigma=self.gaussian_sigma)
        
        # Finde Bereiche mit niedriger Dichte (potenzielle Löcher)
        # Verwende dynamischen Schwellwert basierend auf Dichte-Verteilung
        if np.any(smoothed > 0):
            # Für kleine Löcher: aggressiverer Schwellwert
            threshold = np.percentile(smoothed[smoothed > 0], 15)  # Untere 15% = Lücken
        else:
            return []
        
        # Binäre Maske: True = Loch (niedrige Dichte)
        hole_mask = smoothed < threshold
        
        # Sanftere morphologische Operationen für kleine Löcher (20cm)
        # Erosion entfernt einzelne Pixel-Rauschen, dann Dilation stellt Form wieder her
        hole_mask = binary_erosion(hole_mask, iterations=1)
        hole_mask = binary_dilation(hole_mask, iterations=1)
        
        # Finde zusammenhängende Loch-Regionen
        labeled_array, num_features = label(hole_mask)
        
        self.get_logger().info(f"Gefundene Loch-Kandidaten (2D-Projektion): {num_features}")
        
        hole_regions = []
        for region_id in range(1, num_features + 1):
            # Grid-Indizes dieser Region
            region_cells = np.argwhere(labeled_array == region_id)
            
            # Dynamische Mindestgröße basierend auf erwarteter Lochgröße
            # 20cm Loch bei 1cm Voxel = theoretisch 20x20 = 400 Zellen
            # Aber durch Okklusion, Glättung und morphologische Ops: Faktor ~0.1-0.3
            # Also: min_region_cells = 15 entspricht ca. 15-20cm tatsächlichem Loch
            if len(region_cells) < self.min_region_cells:
                self.get_logger().debug(
                    f"Region verworfen: zu wenig Zellen ({len(region_cells)} < {self.min_region_cells})"
                )
                continue
            
            # Zurück zu Weltkoordinaten (XY)
            region_x_min = region_cells[:, 0].min() * grid_resolution_2d + xy_bounds[0]
            region_x_max = region_cells[:, 0].max() * grid_resolution_2d + xy_bounds[0]
            region_y_min = region_cells[:, 1].min() * grid_resolution_2d + xy_bounds[2]
            region_y_max = region_cells[:, 1].max() * grid_resolution_2d + xy_bounds[2]
            
            # Durchmesser der Region bestimmen
            diameter_x = region_x_max - region_x_min
            diameter_y = region_y_max - region_y_min
            avg_diameter = (diameter_x + diameter_y) / 2.0
            
            # Filter: Größencheck
            if avg_diameter < self.min_hole_diameter or avg_diameter > self.max_hole_diameter:
                self.get_logger().debug(
                    f"Region verworfen: Durchmesser {avg_diameter:.2f}m außerhalb "
                    f"[{self.min_hole_diameter}, {self.max_hole_diameter}]"
                )
                continue
            
            # Bestimme die vertikale Ausdehnung dieser Region
            # Finde alle Punkte innerhalb der XY-Bounding-Box
            in_region_x = (points[:, 0] >= region_x_min) & (points[:, 0] <= region_x_max)
            in_region_y = (points[:, 1] >= region_y_min) & (points[:, 1] <= region_y_max)
            in_region = in_region_x & in_region_y
            
            if not np.any(in_region):
                continue
            
            region_points = points[in_region]
            region_z_min = np.min(region_points[:, 2])
            region_z_max = np.max(region_points[:, 2])
            
            # Zentroid
            centroid_x = (region_x_min + region_x_max) / 2.0
            centroid_y = (region_y_min + region_y_max) / 2.0
            centroid_z = (region_z_min + region_z_max) / 2.0
            
            hole_regions.append({
                'centroid': np.array([centroid_x, centroid_y, centroid_z]),
                'bounds_min': np.array([region_x_min, region_y_min, region_z_min]),
                'bounds_max': np.array([region_x_max, region_y_max, region_z_max]),
                'diameter': avg_diameter,
                'voxel_count': len(region_cells)
            })
        
        self.get_logger().info(f"Gefundene gültige Loch-Regionen: {len(hole_regions)}")
        return hole_regions
    
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
            from scipy.ndimage import gaussian_filter
        except ImportError:
            gaussian_filter = numpy_gaussian_filter
            self.get_logger().debug("Verwende NumPy-Fallback für gaussian_filter")
        
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
            label = numpy_label
            self.get_logger().debug("Verwende NumPy-Fallback für label")
        
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
    
    def analyze_hole_region(
        self, region: dict, header: Header
    ) -> Optional[Entrance]:
        """
        Analysiert eine gefundene Loch-Region.
        """
        centroid = region['centroid']
        bounds_min = region['bounds_min']
        bounds_max = region['bounds_max']
        diameter = region.get('diameter', None)
        
        # Dimensionen berechnen
        height = bounds_max[2] - bounds_min[2]
        x_range = bounds_max[0] - bounds_min[0]
        y_range = bounds_max[1] - bounds_min[1]
        
        # Verwende Durchmesser falls vorhanden, sonst max(x_range, y_range)
        if diameter is not None:
            width = diameter
        else:
            width = max(x_range, y_range)
        
        # Tiefenfilter - nur nahe Löcher
        distance_from_robot = np.sqrt(centroid[0]**2 + centroid[1]**2)
        if distance_from_robot > 2.5:
            self.get_logger().debug(
                f"Region verworfen: zu weit ({distance_from_robot:.2f}m > 2.5m)"
            )
            return None
        
        # Größenfilter
        if height < self.height_threshold or width < self.width_threshold or width > self.max_width:
            return None
        
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
    
    def publish_region_markers(self, regions: List[dict], header: Header):
        """Visualisiert Loch-Regionen"""
        marker_array = MarkerArray()
        
        for i, region in enumerate(regions):
            marker = Marker()
            marker.header = header
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            centroid = region['centroid']
            bounds_min = region['bounds_min']
            bounds_max = region['bounds_max']
            
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = float(bounds_max[0] - bounds_min[0])
            marker.scale.y = float(bounds_max[1] - bounds_min[1])
            marker.scale.z = float(bounds_max[2] - bounds_min[2])
            
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.3
            
            marker_array.markers.append(marker)
        
        self.region_marker_pub.publish(marker_array)
    
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
