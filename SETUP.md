# 🚀 Setup & Installation Guide
## Unitree GO2 Höhlenforscher - ROS2 System

Diese Anleitung führt dich durch die komplette Installation aller benötigten Komponenten.

---

## 📋 Voraussetzungen

- **Ubuntu 22.04 LTS** (auf dem Unitree GO2 oder Entwicklungsrechner)
- **Python 3.10+**
- **Sudo-Rechte** für System-Pakete
- **Internetverbindung** für Downloads

---

## 1️⃣ ROS2 Humble Installation

### Schritt 1.1: System vorbereiten
```bash
# System aktualisieren
sudo apt update
sudo apt upgrade -y

# Lokalisierung einrichten
sudo apt install -y locales
sudo locale-gen de_DE.UTF-8
```

### Schritt 1.2: ROS2 Repository hinzufügen
```bash
# Universe Repository aktivieren
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# ROS2 GPG Key hinzufügen
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2 Repository hinzufügen
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Schritt 1.3: ROS2 Humble installieren
```bash
# Paketlisten aktualisieren
sudo apt update

# ROS2 Humble Desktop installieren (mit RViz, rqt, etc.)
sudo apt install -y ros-humble-desktop

# ROS2 Development Tools
sudo apt install -y ros-dev-tools

# Python3 Colcon (Build-Tool)
sudo apt install -y python3-colcon-common-extensions
```

### Schritt 1.4: ROS2 Environment einrichten
```bash
# ROS2 in .bashrc hinzufügen (automatisch bei jedem Terminal)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verifizieren
ros2 --version
# Erwartete Ausgabe: "ros2 cli version: 0.X.X"
```

---

## 2️⃣ Unitree ROS2 SDK Installation

### Schritt 2.1: Notwendige Build-Tools
```bash
sudo apt install -y build-essential cmake git
```

### Schritt 2.2: Unitree ROS2 SDK klonen
```bash
# ROS2 Workspace erstellen (falls nicht vorhanden)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Unitree ROS2 SDK klonen
git clone https://github.com/unitreerobotics/unitree_ros2.git

# Cyclone DDS installieren (empfohlener DDS für Unitree)
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# Cyclone DDS als Standard setzen
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### Schritt 2.3: Unitree ROS2 bauen
```bash
# Zurück zum Workspace
cd ~/ros2_ws

# Dependencies installieren
rosdep install --from-paths src --ignore-src -r -y

# Workspace bauen
colcon build --symlink-install

# Workspace sourcen
source ~/ros2_ws/install/setup.bash

# In .bashrc hinzufügen
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Schritt 2.4: Verbindung zum GO2 testen
```bash
# GO2 Topics anzeigen (GO2 muss eingeschaltet sein!)
ros2 topic list

# Erwartete Topics:
# /utlidar/cloud        (LiDAR Point Cloud)
# /camera/color         (Kamera)
# /robot_state          (Roboter Status)
# /cmd_vel              (Bewegungskommandos)
```

---

## 3️⃣ RealSense SDK (Depth Camera)

### Schritt 3.1: Intel RealSense Repository
```bash
# Repository-Schlüssel hinzufügen
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# RealSense SDK installieren
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

### Schritt 3.2: Python Wrapper
```bash
pip3 install pyrealsense2
```

### Schritt 3.3: Kamera testen
```bash
# RealSense Viewer starten (GUI)
realsense-viewer

# Oder via Python
python3 -c "import pyrealsense2 as rs; print('RealSense SDK OK')"
```

---

## 4️⃣ Python-Abhängigkeiten

### Schritt 4.1: ROS2 Python-Pakete
```bash
sudo apt install -y \
    python3-rclpy \
    python3-cv-bridge \
    python3-sensor-msgs \
    python3-geometry-msgs \
    python3-std-msgs
```

### Schritt 4.2: Computer Vision & ML
```bash
pip3 install \
    opencv-python \
    numpy \
    scipy \
    torch \
    ultralytics
```

### Schritt 4.3: Weitere Abhängigkeiten
```bash
pip3 install \
    pyyaml \
    transforms3d
```

---

## 5️⃣ Point Cloud Verarbeitung

### Schritt 5.1: Point Cloud Library (PCL)
```bash
# PCL für Point Cloud Verarbeitung
sudo apt install -y \
    libpcl-dev \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions

# ROS2 Point Cloud Tools
sudo apt install -y \
    ros-humble-perception-pcl \
    ros-humble-pcl-msgs
```

### Schritt 5.2: LiDAR Topics testen
```bash
# Unitree LiDAR Topic (wird automatisch von unitree_ros2 bereitgestellt)
ros2 topic echo /utlidar/cloud --once

# Point Cloud in RViz2 visualisieren
rviz2
# In RViz2: Add -> PointCloud2 -> Topic: /utlidar/cloud
```

---

## 6️⃣ Workspace einrichten

### Schritt 6.1: Projekt klonen
```bash
# Falls noch nicht geschehen
cd ~/Documents/GitHub
git clone <YOUR_REPO_URL> unitree_go2_hoehlenforscher
cd unitree_go2_hoehlenforscher
```

### Schritt 6.2: ROS2 Workspace bauen
```bash
# Im Projektverzeichnis
colcon build --symlink-install

# Workspace sourcen
source install/setup.bash

# In .bashrc hinzufügen (optional, für permanente Aktivierung)
echo "source ~/Documents/GitHub/unitree_go2_hoehlenforscher/install/setup.bash" >> ~/.bashrc
```

---

## 7️⃣ Konfiguration

### Schritt 7.1: Config-Dateien anpassen
```bash
# Navigiere zu Konfigurationsverzeichnis
cd ros2_nodes/config

# Bearbeite die YAML-Dateien nach Bedarf:
nano depth_camera_config.yaml
nano unitree_lidar_config.yaml
nano verarbeitung_config.yaml
```

### Schritt 7.2: Netzwerk-Einstellungen (Unitree GO2)
```bash
# Stelle sicher, dass der GO2 im gleichen Netzwerk ist
# Standard-IP des GO2: 192.168.123.15

# Ping testen
ping 192.168.123.15
```

---

## 8️⃣ System testen

### Schritt 8.1: Einzelne Nodes starten
```bash
# Terminal 1: Depth Camera Node
ros2 run unitree_navigation depth_camera_node

# Terminal 2: LiDAR Node
ros2 run unitree_navigation unitree_lidar_node

# Terminal 3: Verarbeitungs-Node
ros2 run unitree_navigation verarbeitung_node

# Terminal 4: Steuerungs-Node
ros2 run unitree_navigation steuerung_node
```

### Schritt 8.2: Mit Launch-File starten
```bash
# Alle Nodes gleichzeitig
ros2 launch unitree_navigation full_system.launch.py

# Oder nur Sensoren
ros2 launch unitree_navigation sensors.launch.py
```

### Schritt 8.3: Topics überprüfen
```bash
# Alle aktiven Topics anzeigen
ros2 topic list

# Depth Camera Topic testen
ros2 topic echo /sensor/depth_image --once

# LiDAR Topic testen
ros2 topic echo /sensor/lidar_pointcloud --once

# Erkannte Eingänge
ros2 topic echo /detection/entrances
```

---

## 🔧 Troubleshooting

### Problem: RealSense Kamera nicht erkannt
```bash
# USB-Verbindung prüfen
lsusb | grep Intel

# Berechtigungen setzen
sudo usermod -aG video $USER
# Neu anmelden erforderlich!
```

### Problem: "ModuleNotFoundError: No module named 'rclpy'"
```bash
# ROS2 Environment sourcen
source /opt/ros/humble/setup.bash
source ~/Documents/GitHub/unitree_go2_hoehlenforscher/install/setup.bash
```

### Problem: Unitree SDK nicht gefunden
```bash
# Prüfe ob SDK gebaut wurde
ls ~/ros2_ws/src/unitree_ros2

# Workspace source prüfen
source ~/ros2_ws/install/setup.bash

# Verifizieren dass Packages verfügbar sind
ros2 pkg list | grep unitree
```

### Problem: GO2 Topics nicht sichtbar
```bash
# Netzwerk-Verbindung zum GO2 prüfen
ping 192.168.123.15

# DDS Discovery prüfen
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list

# Firewall deaktivieren (temporär)
sudo ufw disable

# Multicast-Route hinzufügen (falls nötig)
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev <INTERFACE>
# Ersetze <INTERFACE> mit deiner Netzwerkschnittstelle (z.B. eth0, wlan0)
```

---

## ✅ Installations-Checkliste

- [ ] Ubuntu 22.04 LTS installiert
- [ ] ROS2 Humble installiert und getestet
- [ ] Unitree SDK2 gebaut
- [ ] RealSense SDK installiert
- [ ] Python-Abhängigkeiten installiert
- [ ] PCL installiert
- [ ] Projekt geklont und gebaut
- [ ] Config-Dateien angepasst
- [ ] Netzwerk zum GO2 getestet
- [ ] Einzelne Nodes erfolgreich gestartet
- [ ] Topics verifiziert

---

## 📚 Nützliche Befehle

```bash
# ROS2 Topics anzeigen
ros2 topic list

# Node-Informationen
ros2 node list
ros2 node info /depth_camera_node

# Parameter anzeigen
ros2 param list /depth_camera_node
ros2 param get /depth_camera_node frame_width

# Logs anzeigen
ros2 run rqt_console rqt_console

# Visualisierung (Point Clouds, Images)
rviz2
```

---

**🎉 Fertig! Dein System ist jetzt einsatzbereit.**

Bei Fragen siehe die einzelnen README-Dateien im Projekt oder die [ROS2 Dokumentation](https://docs.ros.org/en/humble/).
