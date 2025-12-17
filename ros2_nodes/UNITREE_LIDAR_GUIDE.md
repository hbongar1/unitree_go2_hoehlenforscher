# Unitree 4D LiDAR L1 Integration

## ✅ Angepasst für Ihren Unitree GO2 LiDAR!

Der Node `unitree_lidar_node.py` ist jetzt speziell für den **Unitree 4D LiDAR L1** konfiguriert.

## 📡 Wie es funktioniert

### 1. Topic-Verbindung
```
Unitree LiDAR → /utlidar/cloud (PointCloud2) → unitree_lidar_node.py
```

Der Unitree 4D LiDAR L1 sendet **3D Punktwolken** auf `/utlidar/cloud`.

### 2. Verarbeitung
Der Node:
1. **Extrahiert 2D-Slice** auf Bodenhöhe (-30cm bis +30cm)
2. **Findet Lücken** in diesem 2D-Slice (Eingänge!)
3. **Published LaserScan** für Fusion mit Kamera

### 3. Integration
```
unitree_lidar_node → /sensor/lidar_scan → verarbeitung_node (Fusion!)
```

## 🚀 Verwendung

### Starten Sie den Node:

```bash
# Terminal 1: Unitree LiDAR Node
python3 ros2_nodes/unitree_lidar_node.py

# Terminal 2: Kamera Node
python3 ros2_nodes/sensor_node.py

# Terminal 3: Verarbeitung mit Fusion
python3 ros2_nodes/verarbeitung_node.py
```

### Prüfen Sie die Verbindung:

```bash
# Schauen Sie ob PointCloud empfangen wird:
ros2 topic echo /utlidar/cloud --once

# Schauen Sie ob LaserScan gepublished wird:
ros2 topic echo /sensor/lidar_scan --once
```

## ⚙️ Parameter

In `unitree_lidar_node.py`:

```yaml
slice_height_min: -0.3   # Untere Grenze (30cm unter LiDAR)
slice_height_max: 0.3    # Obere Grenze (30cm über LiDAR)
min_gap_width: 0.7       # Min. Eingangsbreite
max_gap_width: 2.5       # Max. Eingangsbreite
max_range: 5.0           # Max. Erkennungsreichweite
```

**Tipp:** Passen Sie `slice_height` an die Montagehöhe des LiDAR am GO2 an!

## 🔍 Visualisierung in RViz2

```bash
rviz2

# Fügen Sie hinzu:
# 1. PointCloud2 Display für /utlidar/cloud (Original 3D)
# 2. LaserScan Display für /sensor/lidar_scan (2D Slice)
# 3. Image Display für /sensor/color_image (Kamera)
```

## ⚡ Vorteile vs. 2D LiDAR

| Feature | 2D LiDAR (RPLidar) | **Unitree 4D LiDAR L1** |
|---------|-------------------|------------------------|
| Dimensionen | 2D (nur horizontal) | **3D Punktwolke** ✨ |
| Höhen-Info | ❌ Keine | ✅ **Ja! (z-Koordinate)** |
| Integration | Externe USB-Verbindung | **Bereits am GO2 verbaut** ✨ |
| Reichweite | ~12m | ~150m 🚀 |
| Auflösung | 360 Punkte | **Millionen Punkte** ✨ |
| ROS2 Support | Manuell | **Native Unitree Integration** ✨ |

## 🎯 Nächste Schritte

1. **Starten Sie den Unitree GO2** mit LiDAR aktiviert

2. **Prüfen Sie das Topic:**
   ```bash
   rostopic list | grep utlidar
   # Sollte /utlidar/cloud zeigen
   ```

3. **Starten Sie die Nodes:**
   ```bash
   python3 ros2_nodes/unitree_lidar_node.py
   ```

4. **Schauen Sie die Logs:**
   - "Gap: 1.20m @ 5.3° (dist: 2.50m)" = **Eingang erkannt!** ✅

## 💡 Troubleshooting

### "No messages on /utlidar/cloud"
1. LiDAR am GO2 aktiviert?
2. ROS2-Bridge läuft?
3. Prüfen Sie: `ros2 topic list`

### "Too many/few gaps detected"
→ Passen Sie `slice_height_min/max` an:
- LiDAR zu hoch? → Senken Sie beide Werte
- LiDAR zu niedrig? → Erhöhen Sie beide Werte

### Perfekte 3D-Nutzung (Advanced)
Falls Sie **volle 3D-Höhenmessung** wollen:
- Nutzen Sie die komplette PointCloud (nicht nur 2D-Slice)
- Erkennen Sie Eingangshöhe direkt aus z-Koordinaten
- **Noch präziser als die RealSense Kamera!**

## 🤖 Integration mit Unitree SDK

Der LiDAR läuft über das Unitree ROS2-System. Stellen Sie sicher:
```python
# In Setup.py ist bereits die Basis:
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
ChannelFactoryInitialize(0, "enp129s0")  # Ihr Interface
```

Der LiDAR nutzt **das gleiche Netzwerk** wie die Motor-Steuerung!

---

**Sie haben jetzt die optimale Sensor-Kombination:**
- 👁️ RealSense Kamera (3D, präzise bis 5m)
- 📡 Unitree 4D LiDAR L1 (3D, Reichweite bis 150m!)
- 🧠 Sensor Fusion für maximale Zuverlässigkeit

**Viel Erfolg! 🚀**
