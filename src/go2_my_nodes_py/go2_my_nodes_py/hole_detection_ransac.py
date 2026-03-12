"""
Entrance detection via RANSAC line fitting for precise edge measurements.
Ransac helps find wall edges around openings to measure exact dimensions.
"""

import rclpy
from typing import List, Tuple, Optional
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
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from scipy.ndimage import gaussian_filter, label, sobel


class HoleDetectionRansacNode(BaseNode):
    """
    Detects entrances using RANSAC line fitting on detected edges.

    Pipeline:
    1. Receive PointCloud2
    2. Filter points (±35° FOV, 0.1–1.9 m range)
    3. Find hole candidates via 2D density grid and Sobel edges
    4. For each candidate: fit lines to wall edges surrounding the hole
    5. Measure distance between lines = precise dimension estimates
    6. Publish: ROS topics, RViz markers, CSV log
    """
    
    def __init__(self):
        super().__init__('hole_detection_ransac')
        
        # --- Hole size thresholds ---
        self.min_hole_diameter = 0.08  # minimum width (8 cm)
        self.max_hole_diameter = 2.0   # maximum width (2 m)
        self.height_threshold = 0.1    # minimum height (10 cm)
        self.width_threshold = 0.1     # minimum width  (10 cm)

        # --- 2D density grid (for candidate detection only) ---
        self.voxel_size = 0.005        # grid cell size: 5 mm
        self.gaussian_sigma = 1.0      # smoothing for density map
        self.min_region_cells = 100    # minimum cells for a valid candidate
        self.min_surrounded_sides = 2  # minimum bordered sides for validity

        # --- RANSAC edge fitting ---
        self.ransac_iterations = 150      # iterations for RANSAC
        self.ransac_threshold = 0.01      # 1 cm inlier tolerance
        self.edge_margin_y = 0.01         # margin for width edges (1 cm)
        self.edge_margin_z = 0.05         # margin for height edges (5 cm)

        # --- Multi-frame accumulation ---
        self.frame_buffer_size = 200
        self.point_buffer = deque(maxlen=self.frame_buffer_size)
        self.frame_counter = 0
        self.process_every_n_frames = 1

        # --- Confidence tracking ---
        self.entrance_history = {}
        self.confidence_threshold = 3  # detections needed to publish as stable
        self.entrance_timeout = 60     # frames before an unseen entrance is removed
        self.next_entrance_id = 0
        
        # --- CSV logging ---
        log_dir = os.path.join(os.path.dirname(__file__), 'entrance_detections')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = os.path.join(log_dir, f'ransac_{timestamp}.csv')
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Position_X_m', 'Position_Y_m', 'Position_Z_m',
                             'Width_m', 'Height_m', 'Depth_m', 'Confidence'])
        self.get_logger().info(f"CSV logging enabled: {self.csv_file}")

        self.csv_entry_count = 0
        self.csv_max_entries = 100
        self.csv_width_values = []
        self.csv_height_values = []
        self.csv_depth_values = []
        
        # === ROS2 SETUP ===
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.cloud_sub = self.create_subscription(
            PointCloud2, 'utlidar/cloud_deskewed', self.cloud_data_callback, qos_profile
        )
        
        # Publishers
        self.entrance_pub = self.create_publisher(Entrance, '/entrance_detection_ransac', 10)
        self.filtered_cloud_pub = self.create_publisher(PointCloud2, '/filtered_cloud_ransac', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/entrance_markers_ransac', 10)
        
        self.get_logger().info("HoleDetectionRansacNode started.")
    
    # ====================================================================
    # MAIN CALLBACK
    # ====================================================================
    
    def cloud_data_callback(self, msg: PointCloud2):
        """Main callback: extract, filter, accumulate, and detect entrances."""
        try:
            points = self.extract_points_from_cloud(msg)
            if len(points) == 0:
                return

            filtered_points = self.filter_points(points)
            if len(filtered_points) < 50:
                return

            self.publish_filtered_cloud(filtered_points, msg.header)

            self.point_buffer.append(filtered_points)
            self.frame_counter += 1

            if self.frame_counter % self.process_every_n_frames != 0:
                return

            if len(self.point_buffer) < 200:
                return

            all_points = np.vstack(list(self.point_buffer))
            hole_candidates = self.find_hole_candidates(all_points)

            current_entrances = []
            for candidate in hole_candidates:
                entrance = self.measure_hole_with_ransac(candidate, all_points, msg.header)
                if entrance:
                    current_entrances.append(entrance)

            self.update_entrance_tracking(current_entrances)
            self.publish_stable_entrances(msg.header)

        except Exception as e:
            self.get_logger().error(f"Error in cloud callback: {e}")
    
    # ====================================================================
    # POINT EXTRACTION & FILTERING
    # ====================================================================

    def extract_points_from_cloud(self, cloud: PointCloud2) -> np.ndarray:
        """Extract finite XYZ points from a PointCloud2 message."""
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
        """Filter points to the relevant forward-facing volume: ±35° FOV, 0.1–1.9 m range."""
        if len(points) == 0:
            return points

        angles = np.arctan2(points[:, 1], points[:, 0]) * 180.0 / np.pi
        distances_xy = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        
        angle_filter = np.abs(angles) < 35.0
        distance_filter = (distances_xy > 0.1) & (distances_xy < 1.9)
        z_filter = (points[:, 2] > 0.0) & (points[:, 2] < 3.0)
        
        return points[angle_filter & distance_filter & z_filter]
    
    # ====================================================================
    # CANDIDATE DETECTION (2D density grid + Sobel edges)
    # ====================================================================

    def find_hole_candidates(self, points: np.ndarray) -> List[dict]:
        """Find rough hole candidates using 2D front projection and edge detection."""
        if len(points) < 50:
            return []

        in_front = (points[:, 0] >= 0.3) & (points[:, 0] <= 2.0)
        front_points = points[in_front]

        if len(front_points) < 50:
            return []

        y_points = front_points[:, 1]
        z_points = front_points[:, 2]

        y_min, y_max = np.min(y_points), np.max(y_points)
        z_min, z_max = np.min(z_points), np.max(z_points)

        grid_resolution = self.voxel_size
        y_bins = int((y_max - y_min) / grid_resolution) + 1
        z_bins = int((z_max - z_min) / grid_resolution) + 1

        # Fall back to coarser resolution if grid would be too large
        if y_bins > 500 or z_bins > 500:
            grid_resolution = 0.02
            y_bins = int((y_max - y_min) / grid_resolution) + 1
            z_bins = int((z_max - z_min) / grid_resolution) + 1

        density_2d = np.zeros((y_bins, z_bins), dtype=np.float32)
        y_indices = ((y_points - y_min) / grid_resolution).astype(int)
        z_indices = ((z_points - z_min) / grid_resolution).astype(int)

        for yi, zi in zip(y_indices, z_indices):
            if 0 <= yi < y_bins and 0 <= zi < z_bins:
                density_2d[yi, zi] += 1

        smoothed = gaussian_filter(density_2d, sigma=self.gaussian_sigma)

        # Sobel edge detection
        sx = sobel(smoothed, axis=0, mode='constant')
        sy = sobel(smoothed, axis=1, mode='constant')
        sob = np.hypot(sx, sy)

        edges = sob > np.percentile(sob, 90)

        if np.any(smoothed > 0):
            density_threshold = np.percentile(smoothed[smoothed > 0], 25)
        else:
            return []

        low_density = smoothed < density_threshold
        hole_mask = low_density & ~edges

        labeled_array, num_features = label(hole_mask)

        candidates = []
        for region_id in range(1, num_features + 1):
            region_cells = np.argwhere(labeled_array == region_id)
            
            if len(region_cells) < self.min_region_cells:
                continue

            # Rough bounding box (will be refined by RANSAC)
            region_y_min = region_cells[:, 0].min() * grid_resolution + y_min
            region_y_max = region_cells[:, 0].max() * grid_resolution + y_min
            region_z_min = region_cells[:, 1].min() * grid_resolution + z_min
            region_z_max = region_cells[:, 1].max() * grid_resolution + z_min

            # Require at least N neighbouring sides to have points
            margin = grid_resolution * 5
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
            
            if sum([left, right, top, bottom]) < self.min_surrounded_sides:
                continue
            
            candidates.append({
                'bounds_y': (region_y_min, region_y_max),
                'bounds_z': (region_z_min, region_z_max),
                'front_points': front_points
            })
        
        return candidates
    
    # ====================================================================
    # RANSAC LINE FITTING (precise measurement)
    # ====================================================================

    def measure_hole_with_ransac(self, candidate: dict, all_points: np.ndarray, header: Header) -> Optional[dict]:
        """Measure hole dimensions precisely by fitting lines to wall edges."""
        
        y_min_rough, y_max_rough = candidate['bounds_y']
        z_min_rough, z_max_rough = candidate['bounds_z']
        front_points = candidate['front_points']

        margin_y = self.edge_margin_y
        margin_z = self.edge_margin_z

        # Find points at the 4 edges of the hole
        left_mask = (front_points[:, 1] >= y_min_rough - margin_y*2) & \
                    (front_points[:, 1] <= y_min_rough + margin_y) & \
                    (front_points[:, 2] >= z_min_rough) & \
                    (front_points[:, 2] <= z_max_rough)
        left_points = front_points[left_mask]

        right_mask = (front_points[:, 1] >= y_max_rough - margin_y) & \
                     (front_points[:, 1] <= y_max_rough + margin_y*2) & \
                     (front_points[:, 2] >= z_min_rough) & \
                     (front_points[:, 2] <= z_max_rough)
        right_points = front_points[right_mask]

        top_mask = (front_points[:, 2] >= z_max_rough - margin_z) & \
                   (front_points[:, 2] <= z_max_rough + margin_z*2) & \
                   (front_points[:, 1] >= y_min_rough) & \
                   (front_points[:, 1] <= y_max_rough)
        top_points = front_points[top_mask]

        bottom_mask = (front_points[:, 2] >= z_min_rough - margin_z*2) & \
                      (front_points[:, 2] <= z_min_rough + margin_z) & \
                      (front_points[:, 1] >= y_min_rough) & \
                      (front_points[:, 1] <= y_max_rough)
        bottom_points = front_points[bottom_mask]

        min_edge_points = 5
        if len(left_points) < min_edge_points or len(right_points) < min_edge_points:
            width = self.measure_with_percentile(front_points, y_min_rough, y_max_rough,
                                                   z_min_rough, z_max_rough, axis='y')
        else:
            left_edge  = self.fit_edge_ransac(left_points[:, 1])
            right_edge = self.fit_edge_ransac(right_points[:, 1])
            width = right_edge - left_edge
        
        if len(top_points) < min_edge_points or len(bottom_points) < min_edge_points:
            height = self.measure_with_percentile(front_points, y_min_rough, y_max_rough,
                                                   z_min_rough, z_max_rough, axis='z')
        else:
            top_edge    = self.fit_edge_ransac(top_points[:, 2])
            bottom_edge = self.fit_edge_ransac(bottom_points[:, 2])
            height = top_edge - bottom_edge

        # Validate
        if width < self.min_hole_diameter or width > self.max_hole_diameter:
            return None
        if height < self.height_threshold:
            return None

        # Depth (X extent)
        in_region_y = (front_points[:, 1] >= y_min_rough) & (front_points[:, 1] <= y_max_rough)
        in_region_z = (front_points[:, 2] >= z_min_rough) & (front_points[:, 2] <= z_max_rough)
        in_region = in_region_y & in_region_z

        if not np.any(in_region):
            return None

        region_points = front_points[in_region]
        x_min = np.min(region_points[:, 0])
        x_max = np.max(region_points[:, 0])
        depth = x_max - x_min

        # Centroid
        y_center = (y_min_rough + y_max_rough) / 2.0
        z_center = (z_min_rough + z_max_rough) / 2.0
        x_center = (x_min + x_max) / 2.0

        # Refine centroid using edge positions if available
        if len(left_points) >= min_edge_points and len(right_points) >= min_edge_points:
            y_center = (self.fit_edge_ransac(left_points[:, 1]) +
                       self.fit_edge_ransac(right_points[:, 1])) / 2.0

        if len(top_points) >= min_edge_points and len(bottom_points) >= min_edge_points:
            z_center = (self.fit_edge_ransac(bottom_points[:, 2]) +
                       self.fit_edge_ransac(top_points[:, 2])) / 2.0
        
        entrance_msg = Entrance()
        entrance_msg.header = header
        
        pos = Point()
        pos.x = float(x_center)
        pos.y = float(y_center)
        pos.z = float(z_center)
        entrance_msg.position = pos
        
        entrance_msg.width = float(width)
        entrance_msg.height = float(height)
        
        return {
            'entrance': entrance_msg,
            'depth': float(depth),
            'voxel_count': 0
        }
    
    def fit_edge_ransac(self, values: np.ndarray) -> float:
        """
        RANSAC line fitting for 1D edge position.
        Finds the dominant edge position by iteratively sampling and counting inliers.
        """
        if len(values) < 3:
            return np.median(values)

        best_edge = None
        best_inliers = 0

        for _ in range(self.ransac_iterations):
            sample_idx = np.random.randint(0, len(values))
            candidate_edge = values[sample_idx]

            distances = np.abs(values - candidate_edge)
            inliers = np.sum(distances < self.ransac_threshold)

            if inliers > best_inliers:
                best_inliers = inliers
                inlier_mask = distances < self.ransac_threshold
                best_edge = np.median(values[inlier_mask])

        if best_edge is None or best_inliers < 3:
            return np.median(values)

        return best_edge
    
    def measure_with_percentile(self, points: np.ndarray, y_min: float, y_max: float,
                                 z_min: float, z_max: float, axis: str) -> float:
        """Fallback measurement using percentiles when insufficient edge points exist."""
        
        if axis == 'y':
            in_z = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
            relevant = points[in_z]
            if len(relevant) < 10:
                return y_max - y_min

            left  = np.percentile(relevant[:, 1], 5)
            right = np.percentile(relevant[:, 1], 95)
            return right - left
        else:
            in_y = (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
            relevant = points[in_y]
            if len(relevant) < 10:
                return z_max - z_min

            bottom = np.percentile(relevant[:, 2], 5)
            top    = np.percentile(relevant[:, 2], 95)
            return top - bottom
    
    # ====================================================================
    # TRACKING & PUBLISHING
    # ====================================================================

    def update_entrance_tracking(self, current_entrances: List[dict]):
        """Match new detections to known entrances, update confidence and smooth measurements."""
        smoothing_alpha = 0.3
        
        for data in self.entrance_history.values():
            data['frames_since_seen'] += 1
        
        to_delete = [eid for eid, data in self.entrance_history.items()
                    if data['frames_since_seen'] > self.entrance_timeout]
        for eid in to_delete:
            del self.entrance_history[eid]
        
        for entry in current_entrances:
            entrance = entry['entrance']
            depth = entry.get('depth', 0.0)
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
                self.entrance_history[matched_id]['depth'] = depth
                
                prev_width = self.entrance_history[matched_id].get('smooth_width', entrance.width)
                prev_height = self.entrance_history[matched_id].get('smooth_height', entrance.height)
                prev_depth = self.entrance_history[matched_id].get('smooth_depth', depth)
                
                self.entrance_history[matched_id]['smooth_width'] = \
                    (1 - smoothing_alpha) * prev_width + smoothing_alpha * entrance.width
                self.entrance_history[matched_id]['smooth_height'] = \
                    (1 - smoothing_alpha) * prev_height + smoothing_alpha * entrance.height
                self.entrance_history[matched_id]['smooth_depth'] = \
                    (1 - smoothing_alpha) * prev_depth + smoothing_alpha * depth
            else:
                new_id = self.next_entrance_id
                self.next_entrance_id += 1
                self.entrance_history[new_id] = {
                    'position': pos,
                    'entrance': entrance,
                    'depth': depth,
                    'confidence': 1,
                    'frames_since_seen': 0,
                    'smooth_width': entrance.width,
                    'smooth_height': entrance.height,
                    'smooth_depth': depth
                }
    
    def publish_stable_entrances(self, header: Header):
        """Publish entrances that have exceeded the confidence threshold."""
        published_count = 0
        marker_array = MarkerArray()
        
        for entrance_id, data in self.entrance_history.items():
            if data['confidence'] >= self.confidence_threshold:
                entrance = data['entrance']
                
                smooth_width  = data.get('smooth_width',  entrance.width)
                smooth_height = data.get('smooth_height', entrance.height)
                smooth_depth  = data.get('smooth_depth',  data.get('depth', 0.0))

                entrance.header = header
                entrance.width  = float(smooth_width)
                entrance.height = float(smooth_height)

                self.entrance_pub.publish(entrance)
                published_count += 1

                try:
                    if self.csv_entry_count < self.csv_max_entries:
                        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        with open(self.csv_file, 'a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow([
                                ts,
                                f'{entrance.position.x:.4f}',
                                f'{entrance.position.y:.4f}',
                                f'{entrance.position.z:.4f}',
                                f'{smooth_width:.4f}',
                                f'{smooth_height:.4f}',
                                f'{smooth_depth:.4f}',
                                data['confidence']
                            ])
                        self.csv_width_values.append(smooth_width)
                        self.csv_height_values.append(smooth_height)
                        self.csv_depth_values.append(smooth_depth)
                        self.csv_entry_count += 1

                        if self.csv_entry_count >= self.csv_max_entries:
                            self._write_csv_statistics()
                except Exception as e:
                    self.get_logger().warn(f"CSV logging failed: {e}")

                # RViz marker
                marker = Marker()
                marker.header = header
                marker.ns = "entrances_ransac"
                marker.id = entrance_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                marker.pose.position = entrance.position
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 1.0
                marker.pose.orientation.w = 0.0

                marker.scale.x = 0.05  # thin slab along X
                marker.scale.y = smooth_width
                marker.scale.z = smooth_height

                marker.color.r = 0.0
                marker.color.g = 0.8
                marker.color.b = 1.0
                marker.color.a = 0.7

                marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
                marker_array.markers.append(marker)

        if published_count > 0:
            self.marker_pub.publish(marker_array)
            self.get_logger().info(f"Published {published_count} stable entrance(s).")
    
    def publish_filtered_cloud(self, points: np.ndarray, header: Header):
        """Publish filtered points as PointCloud2 for visualization in RViz."""
        if len(points) == 0:
            return

        points = np.ascontiguousarray(points.astype(np.float32))

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True

        cloud_msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * len(points)

        cloud_msg.data = points.ravel().tobytes()
        self.filtered_cloud_pub.publish(cloud_msg)
    
    def _write_csv_statistics(self):
        """Append mean width, height, and depth statistics to the CSV file."""
        try:
            if not self.csv_width_values:
                return

            avg_width  = np.mean(self.csv_width_values)
            avg_height = np.mean(self.csv_height_values)
            avg_depth  = np.mean(self.csv_depth_values)

            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([])
                writer.writerow(['STATISTICS'])
                writer.writerow(['Average Width (m):',  f'{avg_width:.4f}'])
                writer.writerow(['Average Height (m):', f'{avg_height:.4f}'])
                writer.writerow(['Average Depth (m):',  f'{avg_depth:.4f}'])
                writer.writerow(['Total entries:', self.csv_entry_count])

            self.get_logger().info(
                f"CSV statistics written after {self.csv_entry_count} entries: "
                f"width={avg_width:.4f}m, height={avg_height:.4f}m, depth={avg_depth:.4f}m"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV statistics: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HoleDetectionRansacNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
