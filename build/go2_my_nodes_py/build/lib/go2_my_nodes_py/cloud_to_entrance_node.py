import rclpy
from typing import List, Tuple, Dict
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
    """Node zur Verarbeitung von PointCloud2 zu Entrance-Erkennungen mit Multi-Frame-Akkumulation"""
    
    def __init__(self):
        super().__init__(
            name="cloud_to_entrance_node",
            description="Verarbeitet PointCloud2 zu Entrance-Kandidaten mit temporaler Akkumulation"
        )

        # Subscriber mit flexiblem QoS hinzufügen (wichtig für Lidar-Kompatibilität)
        # Versuche zuerst Reliable, dann Best-Effort wenn das fehlschlägt
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Nicht Critical für Sensors
            durability=QoSDurabilityPolicy.VOLATILE  # PointCloud ist flüchtig
        )
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_deskewed',
            self.cloud_data_callback,
            qos_profile
        )

        # Publisher hinzufügen
        self.publisher = self.create_publisher(Entrance, 'detected_entrances', 10)
        
        # Visualisierungs-Publisher
        self.marker_pub = self.create_publisher(MarkerArray, 'entrance_markers', 10)
        self.cluster_marker_pub = self.create_publisher(MarkerArray, 'cluster_markers', 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, 'filtered_cloud', 10)
        self.cluster_cloud_pub = self.create_publisher(PointCloud2, 'clustered_cloud', 10)
        
        # Parameter für die Verarbeitung
        self.height_threshold = 0.2      # Min. Höhe für Eingang (1.8m)
        self.width_threshold = 0.2       # Min. Breite für Eingang (0.8m)
        self.max_width = 1.0             # Max. Breite für Eingang (3.0m)
        self.cluster_distance = 0.5      # Distanz für Clustering (0.5m)
        
        # Multi-Frame-Akkumulation
        self.frame_buffer_size = 20      # Anzahl Frames zum Akkumulieren
        self.point_buffer = deque(maxlen=self.frame_buffer_size)
        self.frame_counter = 0
        self.process_every_n_frames = 5  # Nur alle N Frames verarbeiten
        
        # Konfidenz-Tracking für Eingänge
        self.entrance_history: Dict[int, dict] = {}  # ID -> {position, confidence, last_seen}
        self.confidence_threshold = 2    # Mindestens 2x gesehen
        self.entrance_timeout = 40       # Nach 20 Frames ohne Sichtung entfernen
        self.next_entrance_id = 0
        
        self.get_logger().info(
            f"Initialisiert mit: buffer={self.frame_buffer_size} frames, "
            f"process_every={self.process_every_n_frames}, "
            f"confidence_threshold={self.confidence_threshold}"
        )
    
    def cloud_data_callback(self, msg: PointCloud2):
        """
        Verarbeitet eingehende Punktwolke mit Multi-Frame-Akkumulation
        
        Strategie:
        1. Punkte aus jedem Frame extrahieren und in Buffer sammeln
        2. Nur alle N Frames die akkumulierten Punkte verarbeiten
        3. Eingänge über mehrere Frames tracken (Konfidenz-System)
        4. Nur stabile, häufig gesehene Eingänge publishen
        """
        self.frame_counter += 1
        
        try:
            # 1. Punkte aus aktueller PointCloud extrahieren
            points = self.extract_points_from_cloud(msg)
            
            if points is None or len(points) == 0:
                self.get_logger().warn("Keine gültigen Punkte in Cloud")
                return
            
            # 2. Punkte filtern
            filtered_points = self.filter_points(points)
            
            if len(filtered_points) == 0:
                self.get_logger().debug("Keine Punkte nach Filterung")
                return
            
            # 3. Zu Buffer hinzufügen
            self.point_buffer.append(filtered_points)
            self.get_logger().debug(
                f"Frame {self.frame_counter}: {len(filtered_points)} Punkte, "
                f"Buffer: {len(self.point_buffer)}/{self.frame_buffer_size}"
            )
            
            # 4. Nur alle N Frames verarbeiten (CPU-Optimierung)
            if self.frame_counter % self.process_every_n_frames != 0:
                return
            
            # 5. Alle gepufferten Punkte kombinieren
            if len(self.point_buffer) < 3:
                self.get_logger().info("Warte auf mehr Frames...")
                return
            
            combined_points = np.vstack(list(self.point_buffer))
            self.get_logger().info(
                f"Verarbeite {len(combined_points)} akkumulierte Punkte "
                f"aus {len(self.point_buffer)} Frames"
            )
            
            # 6. Visualisierung: Gefilterte Cloud publishen
            self.publish_filtered_cloud(combined_points, msg.header)
            
            # 7. Clustering auf kombinierten Punkten
            clusters = self.cluster_points(combined_points)
            self.get_logger().info(f"Gefundene Cluster: {len(clusters)}")
            
            # 8. Visualisierung: Cluster-Cloud publishen
            self.publish_clustered_cloud(clusters, msg.header)
            
            # 8.5 Visualisierung: Cluster-Marker publishen
            self.publish_cluster_markers(clusters, msg.header)
            
            # 9. Cluster analysieren und Kandidaten finden
            current_entrances = []
            for i, cluster in enumerate(clusters):
                entrance = self.analyze_cluster_for_entrance(cluster, msg.header)
                if entrance:
                    current_entrances.append(entrance)
                    self.get_logger().debug(
                        f"Cluster {i}: Eingangs-Kandidat bei "
                        f"({entrance.position.x:.2f}, {entrance.position.y:.2f})"
                    )
            
            # 10. Konfidenz-Tracking: Eingänge über Zeit verfolgen
            self.update_entrance_tracking(current_entrances)
            
            # 11. Nur stabile Eingänge publishen
            self.publish_stable_entrances(msg.header)
            
            # 12. Visualisierung: Marker für erkannte Eingänge
            self.publish_entrance_markers(msg.header)
        
        except Exception as e:
            self.get_logger().error(f"Fehler bei Cloud-Verarbeitung: {e}")
            import traceback
            traceback.print_exc()
    
    def extract_points_from_cloud(self, cloud: PointCloud2) -> np.ndarray:
        """
        Extrahiert XYZ-Punkte aus PointCloud2
        
        PointCloud2 speichert Daten als kontinuierliches Byte-Array.
        """
        data = np.frombuffer(cloud.data, dtype=np.uint8)
        point_step = cloud.point_step
        
        # Extrahiere Offset für X, Y, Z aus Fields
        x_offset = y_offset = z_offset = None
        for field in cloud.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if None in (x_offset, y_offset, z_offset):
            self.get_logger().error("X, Y oder Z nicht in Cloud gefunden")
            return None
        
        # Punkte auslesen
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
        """
        Filtert Punkte:
        - Entfernt NaN/Inf (ungültige Werte)
        - Begrenzt auf relevanten Bereich (5m Umkreis, 0.1-3m Höhe)
        """
        # NaN/Inf entfernen
        valid_mask = np.all(np.isfinite(points), axis=1)
        points = points[valid_mask]
        
        if len(points) == 0:
            return points
        
        # Bereich filtern
        x_range = np.abs(points[:, 0]) < 5.0
        y_range = np.abs(points[:, 1]) < 5.0
        z_range = (points[:, 2] > 0.1) & (points[:, 2] < 3.0)
        
        valid_mask = x_range & y_range & z_range
        return points[valid_mask]
    
    def cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """
        Clustering mit DBSCAN-ähnlichem Ansatz
        Gruppiert räumlich nahe Punkte zu Clustern
        """
        if len(points) < 10:
            return []
        
        clusters = []
        used = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if used[i]:
                continue
            
            # Neuen Cluster starten
            current_cluster = [i]
            used[i] = True
            
            # Nachbarn expandieren (BFS-ähnlich)
            j = 0
            while j < len(current_cluster):
                point_idx = current_cluster[j]
                point = points[point_idx]
                
                # Distanzen zu allen anderen Punkten
                distances = np.linalg.norm(points - point, axis=1)
                
                # Nachbarn im Umkreis finden
                neighbors = np.where(
                    (distances < self.cluster_distance) & (~used)
                )[0]
                
                # Alle neuen Nachbarn zum Cluster hinzufügen
                for neighbor_idx in neighbors:
                    if not used[neighbor_idx]:
                        current_cluster.append(neighbor_idx)
                        used[neighbor_idx] = True
                
                j += 1
            
            # Nur größere Cluster behalten (Rauschen filtern)
            if len(current_cluster) > 20:
                clusters.append(points[current_cluster])
        
        return clusters
    
    def analyze_cluster_for_entrance(
        self, cluster: np.ndarray, header: Header
    ) -> Entrance:
        """
        Analysiert Cluster auf Eingangsmerkmale:
        - Hohe Z-Ausdehnung (Höhe ~1.8-3m)
        - Breite X/Y-Ausdehnung (Breite ~0.8-3m)
        """
        # Dimensionen berechnen
        z_min, z_max = np.min(cluster[:, 2]), np.max(cluster[:, 2])
        x_min, x_max = np.min(cluster[:, 0]), np.max(cluster[:, 0])
        y_min, y_max = np.min(cluster[:, 1]), np.max(cluster[:, 1])
        
        height = z_max - z_min
        x_range = x_max - x_min
        y_range = y_max - y_min
        width = max(x_range, y_range)
        
        # Filterung: Ist das ein potenzieller Eingang?
        if height < self.height_threshold:
            return None
        
        if width < self.width_threshold or width > self.max_width:
            return None
        
        # Position (Zentroid des Clusters)
        centroid = np.mean(cluster, axis=0)
        
        # Entrance-Message erstellen
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
        """
        Konfidenz-Tracking: Verfolgt Eingänge über mehrere Frames
        
        - Neue Eingänge erhalten ID und Konfidenz=1
        - Wiederkehrende Eingänge erhöhen Konfidenz
        - Nicht gesehene Eingänge altern aus
        """
        # Alle existierenden Eingänge altern
        for entrance_id in list(self.entrance_history.keys()):
            self.entrance_history[entrance_id]['frames_since_seen'] += 1
            
            # Timeout: Eingang zu lange nicht gesehen
            if self.entrance_history[entrance_id]['frames_since_seen'] > self.entrance_timeout:
                self.get_logger().info(f"Eingang {entrance_id} entfernt (timeout)")
                del self.entrance_history[entrance_id]
        
        # Aktuelle Eingänge verarbeiten
        for entrance in current_entrances:
            pos = entrance.position
            
            # Prüfe ob Eingang bereits bekannt ist (räumliche Nähe)
            matched_id = None
            for entrance_id, data in self.entrance_history.items():
                stored_pos = data['position']
                distance = np.sqrt(
                    (pos.x - stored_pos.x)**2 + 
                    (pos.y - stored_pos.y)**2 + 
                    (pos.z - stored_pos.z)**2
                )
                
                if distance < 0.8:  # Gleicher Eingang wenn < 0.8m entfernt
                    matched_id = entrance_id
                    break
            
            if matched_id is not None:
                # Bekannter Eingang: Konfidenz erhöhen
                self.entrance_history[matched_id]['confidence'] += 1
                self.entrance_history[matched_id]['frames_since_seen'] = 0
                self.entrance_history[matched_id]['entrance'] = entrance  # Update Daten
                self.get_logger().debug(
                    f"Eingang {matched_id}: Konfidenz={self.entrance_history[matched_id]['confidence']}"
                )
            else:
                # Neuer Eingang
                new_id = self.next_entrance_id
                self.next_entrance_id += 1
                self.entrance_history[new_id] = {
                    'position': pos,
                    'entrance': entrance,
                    'confidence': 1,
                    'frames_since_seen': 0
                }
                self.get_logger().info(
                    f"Neuer Eingangs-Kandidat {new_id} bei "
                    f"({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
                )
    
    def publish_stable_entrances(self, header: Header):
        """
        Publisht nur Eingänge mit ausreichender Konfidenz
        (wurden mehrmals in verschiedenen Frames erkannt)
        """
        published_count = 0
        
        for entrance_id, data in self.entrance_history.items():
            if data['confidence'] >= self.confidence_threshold:
                entrance = data['entrance']
                entrance.header = header  # Aktuellen Timestamp verwenden
                
                self.publisher.publish(entrance)
                published_count += 1
                
                self.get_logger().info(
                    f"✓ Eingang {entrance_id} (Konfidenz={data['confidence']}): "
                    f"pos=({entrance.position.x:.2f}, {entrance.position.y:.2f}, "
                    f"{entrance.position.z:.2f}), "
                    f"größe={entrance.width:.2f}×{entrance.height:.2f}m"
                )
        
        if published_count == 0:
            self.get_logger().info("Keine stabilen Eingänge (zu geringe Konfidenz)")
    
    def publish_filtered_cloud(self, points: np.ndarray, header: Header):
        """Publisht gefilterte PointCloud für Visualisierung in RViz"""
        if len(points) == 0:
            return
        
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        # Stelle sicher dass Frame-ID gesetzt ist
        if not cloud_msg.header.frame_id:
            cloud_msg.header.frame_id = 'odom'
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_bigendian = False
        cloud_msg.is_dense = False
        cloud_msg.point_step = 12  # 3 floats (x, y, z) = 12 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        
        # Fields definieren
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Daten als Bytes packen
        cloud_data = []
        for point in points:
            cloud_data.extend(struct.pack('fff', point[0], point[1], point[2]))
        
        cloud_msg.data = bytes(cloud_data)
        self.filtered_cloud_pub.publish(cloud_msg)
    
    def publish_clustered_cloud(self, clusters: List[np.ndarray], header: Header):
        """Publisht Cluster als farbige PointCloud"""
        if len(clusters) == 0:
            return
        
        # Alle Cluster kombinieren
        all_points = []
        for cluster in clusters:
            all_points.extend(cluster.tolist())
        
        if len(all_points) == 0:
            return
        
        points_array = np.array(all_points)
        self.publish_filtered_cloud(points_array, header)
    
    def publish_cluster_markers(self, clusters: List[np.ndarray], header: Header):
        """Publisht Cluster als 3D-Marker mit unterschiedlichen Farben"""
        marker_array = MarkerArray()
        
        # Farben für Cluster (RGB-Werte)
        colors = [
            (1.0, 0.0, 0.0),  # Rot
            (0.0, 1.0, 0.0),  # Grün
            (0.0, 0.0, 1.0),  # Blau
            (1.0, 1.0, 0.0),  # Gelb
            (1.0, 0.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0),  # Cyan
            (1.0, 0.5, 0.0),  # Orange
            (1.0, 0.0, 0.5),  # Pink
            (0.5, 0.0, 1.0),  # Violett
            (0.0, 0.5, 1.0),  # Blau-Cyan
        ]
        
        for cluster_id, cluster in enumerate(clusters):
            # Cluster-Bounding-Box berechnen
            x_min, x_max = np.min(cluster[:, 0]), np.max(cluster[:, 0])
            y_min, y_max = np.min(cluster[:, 1]), np.max(cluster[:, 1])
            z_min, z_max = np.min(cluster[:, 2]), np.max(cluster[:, 2])
            
            # Zentroid
            centroid = np.mean(cluster, axis=0)
            
            # Farbe aus der Liste (zyklisch)
            color_idx = cluster_id % len(colors)
            r, g, b = colors[color_idx]
            
            # Bounding-Box Marker erstellen
            marker = Marker()
            marker.header = header
            if not marker.header.frame_id:
                marker.header.frame_id = 'odom'
            marker.ns = "clusters"
            marker.id = cluster_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position (Zentroid)
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.pose.orientation.w = 1.0
            
            # Größe (Ausdehnung der Bounding-Box)
            marker.scale.x = float(x_max - x_min) if x_max > x_min else 0.1
            marker.scale.y = float(y_max - y_min) if y_max > y_min else 0.1
            marker.scale.z = float(z_max - z_min) if z_max > z_min else 0.1
            
            # Farbe (transparent um Punkte sichtbar zu halten)
            marker.color = ColorRGBA(r=r, g=g, b=b, a=0.3)
            
            marker.lifetime.sec = 2
            marker_array.markers.append(marker)
            
            # Text-Label mit Cluster-ID und Punktzahl
            text_marker = Marker()
            text_marker.header = header
            if not text_marker.header.frame_id:
                text_marker.header.frame_id = 'odom'
            text_marker.ns = "cluster_labels"
            text_marker.id = cluster_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Text in der Mitte des Clusters
            text_marker.pose.position.x = float(centroid[0])
            text_marker.pose.position.y = float(centroid[1])
            text_marker.pose.position.z = float(centroid[2])
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.25  # Text-Höhe
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"C{cluster_id}\n({len(cluster)})"
            text_marker.lifetime.sec = 2
            
            marker_array.markers.append(text_marker)
        
        if len(marker_array.markers) > 0:
            self.cluster_marker_pub.publish(marker_array)
            self.get_logger().info(f"Publisht {len(clusters)} Cluster-Marker")
    
    def publish_entrance_markers(self, header: Header):
        """Erstellt 3D-Marker (Boxen) für erkannte Eingänge in RViz"""
        marker_array = MarkerArray()
        
        for entrance_id, data in self.entrance_history.items():
            if data['confidence'] >= self.confidence_threshold:
                entrance = data['entrance']
                
                # Marker für Eingangs-Box erstellen
                marker = Marker()
                marker.header = header
                # Stelle sicher dass Frame-ID gesetzt ist
                if not marker.header.frame_id:
                    marker.header.frame_id = 'odom'
                marker.ns = "entrances"
                marker.id = entrance_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # Position
                marker.pose.position = entrance.position
                marker.pose.orientation.w = 1.0
                
                # Größe (width, depth=0.1m, height)
                marker.scale.x = float(entrance.width)
                marker.scale.y = 0.1  # Dünne Wand
                marker.scale.z = float(entrance.height)
                
                # Farbe (Grün mit Transparenz, Konfidenz bestimmt Helligkeit)
                alpha = min(1.0, data['confidence'] / 10.0)
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=alpha)
                
                marker.lifetime.sec = 2  # 2 Sekunden Lebensdauer
                
                marker_array.markers.append(marker)
                
                # Text-Label mit ID und Konfidenz
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.ns = "entrance_labels"
                text_marker.id = entrance_id + 1000  # Offset für Text-IDs
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                # Text über dem Eingang
                text_marker.pose.position.x = entrance.position.x
                text_marker.pose.position.y = entrance.position.y
                text_marker.pose.position.z = entrance.position.z + entrance.height / 2 + 0.3
                text_marker.pose.orientation.w = 1.0
                
                text_marker.scale.z = 0.3  # Text-Höhe
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                text_marker.text = f"E{entrance_id}\nC:{data['confidence']}"
                text_marker.lifetime.sec = 2
                
                marker_array.markers.append(text_marker)
        
        # Alle Marker publishen
        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)
            self.get_logger().debug(f"Published {len(marker_array.markers)} markers")
    
    def process(self, data):
        """Process data from Cloud to Entrance"""
        return data


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

