# Sensor Node Umbenennung - Update

## ✅ Änderungen

**Vorher:**
- `sensor_node.py` (unklar - welcher Sensor?)

**Nachher:**
- `depth_camera_node.py` ✨ (klar: RealSense Tiefenkamera)
- `unitree_lidar_node.py` ✨ (klar: Unitree 4D LiDAR)

## 📂 Neue Struktur

```
ros2_nodes/
├── depth_camera_node.py      # RealSense D435 Tiefenkamera
├── unitree_lidar_node.py      # Unitree 4D LiDAR L1
├── verarbeitung_node.py       # Sensor Fusion + Erkennung
├── verhalten_node.py          # State Machine
└── ausfuehrung_node.py        # Motor Control
```

## 🚀 Neue Befehle

### Starten Sie die Depth Camera:
```bash
python3 ros2_nodes/depth_camera_node.py
```

### Starten Sie den LiDAR:
```bash
python3 ros2_nodes/unitree_lidar_node.py
```

### Beide zusammen:
```bash
# Terminal 1:
python3 ros2_nodes/depth_camera_node.py

# Terminal 2:
python3 ros2_nodes/unitree_lidar_node.py

# Terminal 3:
python3 ros2_nodes/verarbeitung_node.py  # Nutzt beide!
```

## 📡 Topics bleiben gleich

Die Topics haben sich **nicht** geändert:
- `/sensor/depth_image` ✅
- `/sensor/color_image` ✅  
- `/sensor/camera_info` ✅
- `/sensor/lidar_scan` ✅

## ✨ Vorteile

1. **Klarere Namen**: Sofort erkennbar welcher Sensor
2. **Bessere Wartbarkeit**: Kein Raten mehr
3. **Konsistente Namensgebung**:
   - `depth_camera_node` = Kamera
   - `unitree_lidar_node` = LiDAR
   - `verarbeitung_node` = Processing
   - `verhalten_node` = Behavior
   - `ausfuehrung_node` = Execution

**Alles fertig!** 🎉
