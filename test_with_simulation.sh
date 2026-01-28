#!/bin/bash
# Test mit simulierten Lidar-Daten

echo "🧪 Test-Setup mit simulierten Daten"
echo "===================================="
echo ""

source /opt/ros/humble/setup.bash
source /workspace/unitree_go2_hoehlenforscher/install/setup.bash

echo "Starte 3 Komponenten:"
echo "  1. Test Cloud Publisher (simulierte Lidar-Daten)"
echo "  2. Cloud-to-Entrance Node (Verarbeitung)"
echo "  3. RViz2 (Visualisierung)"
echo ""
echo "Drücke Ctrl+C zum Beenden"
echo ""

# Cleanup-Funktion
cleanup() {
    echo ""
    echo "🛑 Stoppe alle Prozesse..."
    kill $PUB_PID $NODE_PID $RVIZ_PID 2>/dev/null
    exit 0
}

trap cleanup SIGINT

# 1. Test Publisher starten
echo "[1/3] Starte Test Cloud Publisher..."
python3 /workspace/unitree_go2_hoehlenforscher/test_cloud_publisher.py &
PUB_PID=$!
sleep 2

# 2. Node starten
echo "[2/3] Starte cloud_to_entrance_node..."
ros2 run go2_my_nodes_py cloud_to_entrance_node &
NODE_PID=$!
sleep 2

# 3. RViz starten
echo "[3/3] Starte RViz2..."
rviz2 -d /workspace/unitree_go2_hoehlenforscher/entrance_detection.rviz &
RVIZ_PID=$!

echo ""
echo "✅ Alle Komponenten gestartet!"
echo ""
echo "In RViz solltest du sehen:"
echo "  • Weiße Punkte: Original simulierte Cloud"
echo "  • Blaue Punkte: Gefilterte Cloud"
echo "  • Grüne Box: Erkannter Eingang (nach ~10 Sekunden)"
echo ""

# Warte auf User-Interrupt
wait
