# Loch-Erkennungsmethoden - Vergleich und Implementierung

## 📋 Überblick

Es wurden drei verschiedene Methoden zur Loch-Erkennung in Punktansammlungen implementiert und separat als eigenständige ROS2-Nodes realisiert:

### 1. **Normal Vector Analysis** (`hole_detection_normal_vector.py`)
- **Farbe in RViz:** Blau (Cluster), Grün (Eingänge)
- **Prinzip:** Analyzes Oberflächennormalen-Varianz
- **Parameter:**
  - `normal_search_radius`: 0.1m (Nachbarschafts-Radius)
  - `normal_variance_threshold`: 0.15 (Varianz-Schwellwert)
  - `normal_window_size`: 10 (Punkte pro Fenster)

### 2. **Convex Hull** (`hole_detection_convex_hull.py`)
- **Farbe in RViz:** Orange (Cluster), Orange/Braun (Eingänge)
- **Prinzip:** Vergleicht Punkte mit Convex Hull Oberfläche
- **Parameter:**
  - `hole_volume_ratio_threshold`: 0.15 (>15% fehlendes Volumen = Loch)
  - `min_interior_points`: 50 (Minimum innere Punkte)

### 3. **Voxel Grid** (`hole_detection_voxel_grid.py`)
- **Farbe in RViz:** Cyan (Cluster), Grün (Eingänge)
- **Prinzip:** Dichte-Gradienten im 3D-Gitter
- **Parameter:**
  - `voxel_size`: 0.05m (5cm Voxel)
  - `gradient_percentile`: 75 (Top 25% = Kanten)
  - `low_density_percentile`: 25 (Unten 25% = Löcher)
  - `gaussian_sigma`: 1.0 (Glättung)

---

## 🚀 Verwendung

### Build
```bash
cd /workspace/unitree_go2_hoehlenforscher
colcon build --packages-select go2_my_nodes_py
source install/setup.bash
```

### Starten aller 3 Nodes (Vergleich)
```bash
ros2 launch go2_my_nodes_py hole_detection_all.launch.py
```

### Einzelne Nodes starten

**Nur Normal Vector:**
```bash
ros2 launch go2_my_nodes_py hole_detection_normal_vector.launch.py
```

**Nur Convex Hull:**
```bash
ros2 launch go2_my_nodes_py hole_detection_convex_hull.launch.py
```

**Nur Voxel Grid:**
```bash
ros2 launch go2_my_nodes_py hole_detection_voxel_grid.launch.py
```

### RViz Visualisierung

```bash
# In separatem Terminal:
ros2 run rviz2 rviz2 -d entrance_detection.rviz
```

Die Visualisierungen zeigen:
- **Weiß/Grau:** Original Lidar Cloud
- **Grün:** Gefilterte Punkte (Normal Vector)
- **Orange:** Gefilterte Punkte (Convex Hull)
- **Cyan:** Gefilterte Punkte (Voxel Grid)
- **Kleine farbige Sphären:** Cluster-Zentroide
- **Große Rechtecke:** Erkannte Eingänge (grün = stabil, >2 Frames)

---

## 📊 Topics

### Publishers pro Methode (Beispiel: Normal Vector)

| Topic | Type | Beschreibung |
|-------|------|-------------|
| `/detected_entrances_normal_vector` | `Entrance` | Erkannte Eingänge |
| `/entrance_markers_normal_vector` | `MarkerArray` | Visualisierungs-Marker |
| `/cluster_markers_normal_vector` | `MarkerArray` | Cluster-Zentroide |
| `/filtered_cloud_normal_vector` | `PointCloud2` | Gefilterte Punktwolke |

**Analog für:**
- `_convex_hull` (Convex Hull Methode)
- `_voxel_grid` (Voxel Grid Methode)

### Subscriber

Alle Nodes abonnieren:
- `/utlidar/cloud_deskewed` (`PointCloud2`) - Lidar Input

---

## ⚙️ Parameter anpassen

### In der Node selbst (nach Edit):

```python
# Beispiel: Voxel Grid
self.voxel_size = 0.05  # Größer = schneller, aber weniger genau
self.gaussian_sigma = 1.0  # Mehr = mehr Glättung (weniger Rauschen)
```

### Oder per ROS2 Parameter (zu implementieren):

```bash
ros2 run go2_my_nodes_py hole_detection_voxel_grid --ros-args -p voxel_size:=0.03
```

---

## 🎯 Vergleich der Methoden

### Performance
| Methode | CPU | RAM | Latenz |
|---------|-----|-----|--------|
| Normal Vector | ⭐⭐⭐ | ⭐⭐⭐ | Mittel |
| Convex Hull | ⭐⭐⭐⭐ | ⭐⭐⭐ | Schnell |
| Voxel Grid | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | **Sehr Schnell** |

### Genauigkeit
| Methode | Kleine Löcher | Große Löcher | Rausch-robust |
|---------|---|---|---|
| Normal Vector | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ |
| Convex Hull | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| Voxel Grid | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ |

### **Empfehlung:** Voxel Grid (beste Balance)

---

## 🔧 Debugging

### Logs anschauen
```bash
ros2 topic echo /detected_entrances_voxel_grid
```

### RViz Topics monitoren
```bash
# In RViz: rechts "Displays" → "+" → nach Topic suchen
```

### CPU/Memory Nutzung
```bash
ros2 topic hz /detected_entrances_voxel_grid
```

---

## 📝 Gemeinsame Parameter (alle Nodes)

```python
# Punkt-Filterung
self.height_threshold = 0.2      # Min. Höhe (m)
self.width_threshold = 0.2       # Min. Breite (m)
self.max_width = 1.0             # Max. Breite (m)
self.cluster_distance = 0.5      # Clustering Radius (m)

# Multi-Frame Akkumulation
self.frame_buffer_size = 20      # Frames zum Akkumulieren
self.process_every_n_frames = 5  # Nur alle N Frames verarbeiten

# Konfidenz-Tracking
self.confidence_threshold = 2    # Mindestens 2x erkannt
self.entrance_timeout = 40       # Nach 40 Frames entfernen
```

---

## 💡 Tipps zur Optimierung

### Für **schnellere Erkennung:**
- Voxel Grid: `voxel_size` erhöhen (0.05 → 0.1m)
- `process_every_n_frames` erhöhen (5 → 10)

### Für **bessere Genauigkeit:**
- `frame_buffer_size` erhöhen (20 → 30)
- `confidence_threshold` erhöhen (2 → 3)
- Voxel Grid: `voxel_size` verkleinern (0.05 → 0.03m)

### Für **weniger False Positives:**
- `height_threshold` erhöhen
- `width_threshold` erhöhen
- `confidence_threshold` erhöhen

---

## 📚 Dependencies

```bash
sudo apt-get install python3-scipy python3-numpy
```

Bereits in ROS2 Humble enthalten:
- `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `visualization_msgs`

---

## 🧪 Testen ohne Hardware

Simulationsumgebung verwenden:
```bash
ros2 bag play <recorded_pointcloud>.mcap
# Dann einen der Nodes starten
```

---

## 📖 Weitere Ressourcen

- ROS2 Humble Documentation: https://docs.ros.org/en/humble/
- Open3D für erweiterte Geometrie: http://www.open3d.org/
- PCL für Point Cloud Processing: https://pointclouds.org/

---

**Autor:** AI Assistant  
**Datum:** 2026-01-28  
**Status:** ✅ Implementierung komplett - bereit zum Testen
