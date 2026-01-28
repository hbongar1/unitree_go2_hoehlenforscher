# Wall and Hole Detection für Unitree Go2 mit ROS2 Humble

Dieses Projekt erkennt Wände und 50x50cm Löcher in Wänden mit dem Lidar des Unitree Go2 Roboters.

## Features

✅ **Ebenen-Erkennung**: Verwendet RANSAC um Wände als Ebenen zu identifizieren
✅ **Loch-Detektion**: Findet Löcher (50x50cm) in erkannten Wänden
✅ **ROS2 Integration**: Vollständig mit ROS2 Humble integriert
✅ **Visualisierung**: Rviz2-Marker für Wand und Löcher
✅ **Point Cloud Verarbeitung**: Effiziente 3D-Datenverarbeitung mit Open3D

## Systemanforderungen

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble
- **Python**: 3.10+
- **Dependencies**: Open3D, NumPy, ROS2 Python packages

## Installation

### 1. Dependencies installieren

```bash
bash install_dependencies.sh
```

### 2. ROS2 Workspace aufsetzen

```bash
# Wechsel in den workspace
cd /path/to/ros2_workspace

# Build des Packages
colcon build

# Source setup
source install/setup.bash
```

## Verwendung

### Node direkt starten

```bash
ros2 run entrance_detection wall_hole_detection_node.py
```

### Mit Launch-Datei starten

```bash
ros2 launch entrance_detection wall_hole_detection.launch.py
```

## Topics

### Subscriber
- `/utlidar/cloud` (sensor_msgs/PointCloud2) - Input vom Unitree LiDAR

### Publisher
- `/detection/wall_plane` (sensor_msgs/PointCloud2) - Erkannte Wandpunkte
- `/detection/hole_points` (sensor_msgs/PointCloud2) - Loch-Punkte
- `/detection/wall_hole_markers` (visualization_msgs/MarkerArray) - Rviz2 Visualisierung

## Parameter

Konfigurierbar in `wall_hole_detection_params.yaml`:

| Parameter | Default | Beschreibung |
|-----------|---------|-------------|
| `distance_threshold` | 0.05 | Max. Distanz zur Ebene in Metern |
| `ransac_iterations` | 1000 | Anzahl RANSAC-Iterationen |
| `min_plane_points` | 100 | Min. Punkte für gültige Ebene |
| `hole_width` | 0.50 | Erwartete Loch-Breite (m) |
| `hole_height` | 0.50 | Erwartete Loch-Höhe (m) |
| `hole_min_points` | 20 | Min. Rasterzellen für Loch |

## Algorithmus

### Schritt 1: Ebenen-Erkennung
- RANSAC wird verwendet um die dominante Ebene (Wand) in der Punktwolke zu finden
- Punkte außerhalb eines Distanz-Schwellwerts werden gefiltert

### Schritt 2: Loch-Detektion
- Alle Punkte werden auf die erkannte Ebene projiziert
- Ein 2D-Gitter wird erstellt um Bereiche mit niedriger Punktdichte zu finden
- Bereiche mit < 20% der durchschnittlichen Dichte werden als Löcher erkannt
- Benachbarte Zellen werden geclustert

### Schritt 3: Visualisierung
- Marker werden publiziert für Rviz2-Visualisierung
- Blau: Wandebene
- Rot: Erkannte Löcher

## Debugging

### Logs anschauen
```bash
ros2 run entrance_detection wall_hole_detection_node.py --ros-args --log-level debug
```

### Topics inspizieren
```bash
# Verfügbare Topics anzeigen
ros2 topic list

# PointCloud2 visualisieren
ros2 topic echo /detection/wall_plane

# Marker anzeigen
rviz2
```

## Tipps

- **Genügend Punkte**: Der LiDAR braucht genug Punkte um eine Wand zu erkennen (min. 100)
- **Nähe zur Wand**: Roboter sollte 0.5-5 Meter von der Wand entfernt sein
- **Loch-Größe**: Das Loch sollte mind. 50x50cm groß sein um erkannt zu werden
- **Parameter tunen**: `distance_threshold` und `ransac_iterations` für verschiedene Umgebungen anpassen

## Troubleshooting

| Problem | Lösung |
|---------|--------|
| "Nicht genug Punkte" | Roboter näher an Wand bewegen, mehr Zeit für Datensammlung |
| Wand wird nicht erkannt | `min_plane_points` reduzieren, `distance_threshold` erhöhen |
| Falsche Löcher erkannt | `hole_min_points` erhöhen, Umgebung prüfen |
| Open3D Fehler | `pip3 install --upgrade open3d` |

## Lizenz

MIT

## Autoren

Moritz Schipp - Semester 5 SIR Project
