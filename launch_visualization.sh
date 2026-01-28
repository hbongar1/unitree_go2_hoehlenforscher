#!/bin/bash

# Launch-Script für Eingangs-Erkennung mit Visualisierung

echo "🚀 Starte Eingangs-Erkennungs-System mit Visualisierung..."
echo ""
echo "Dieses Script startet:"
echo "  1. Den cloud_to_entrance_node (Verarbeitung)"
echo "  2. RViz2 (Visualisierung)"
echo ""

# Farben für Terminal-Output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ROS2 Setup sourcen
echo -e "${BLUE}[1/3] Source ROS2 Setup...${NC}"
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Node starten (im Hintergrund)
echo -e "${BLUE}[2/3] Starte cloud_to_entrance_node...${NC}"
ros2 run go2_my_nodes_py cloud_to_entrance_node &
NODE_PID=$!

# Kurz warten damit Node startet
sleep 2

# RViz2 mit Konfiguration starten
echo -e "${BLUE}[3/3] Starte RViz2...${NC}"
echo -e "${GREEN}✓ System gestartet!${NC}"
echo ""
echo "📊 RViz2 Topics:"
echo "  - /utlidar/cloud_deskewed  : Original Lidar-Daten (weiß)"
echo "  - /filtered_cloud          : Gefilterte Punkte (blau)"
echo "  - /entrance_markers        : Erkannte Eingänge (grüne Boxen)"
echo ""
echo "🎯 Eingangs-Kriterien:"
echo "  - Höhe: ≥ 1.8m"
echo "  - Breite: 0.8m - 3.0m"
echo "  - Konfidenz: ≥ 3x erkannt"
echo ""
echo "Press Ctrl+C to stop..."

rviz2 -d entrance_detection.rviz

# Cleanup beim Beenden
kill $NODE_PID 2>/dev/null
echo ""
echo "System beendet."
