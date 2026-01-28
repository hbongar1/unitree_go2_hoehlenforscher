#!/bin/bash
# Test-Skript für Eingangserkennung mit Pappe-Cutouts
# Startet den Voxel Grid Detector mit optimierten Parametern für Pappe-Erkennung

echo "=========================================="
echo "  🔍 Pappe Entrance Detection Test"
echo "=========================================="
echo ""
echo "Dieses Skript startet:"
echo "  ✓ Voxel Grid Hole Detection (optimiert für Pappe)"
echo "  ✓ RViz2 Visualisierung"
echo "  ✓ Live Entrance Detection"
echo ""
echo "Setup-Anforderungen:"
echo "  - Pappe-Aufbau mit Cutouts bereit"
echo "  - Unitree GO2 LiDAR aktiv (/utlidar/cloud_deskewed Topic)"
echo ""
echo "Erwartete Erkennungen (lt. Foto):"
echo "  1. Kleines Loch: ~20cm x 20cm (sollte als zu klein gefiltert werden)"
echo "  2. Richtiges Loch: ~32cm breit, 52-55cm hoch (SOLLTE ERKANNT WERDEN)"
echo "  3. Großes Loch: ~44cm x 44cm (SOLLTE ERKANNT WERDEN)"
echo ""
read -p "Bereit zum Start? [Enter drücken]"

# Baue das Package
echo "Building package..."
cd "$(dirname "$0")"
colcon build --packages-select go2_my_nodes_py --symlink-install

if [ $? -ne 0 ]; then
    echo "❌ Build fehlgeschlagen!"
    exit 1
fi

# Source setup
source install/setup.bash

echo ""
echo "✓ Build erfolgreich!"
echo "🚀 Starte Voxel Grid Entrance Detection..."
echo ""

# Launch mit RViz
ros2 launch go2_my_nodes_py hole_detection_voxel_grid.launch.py

# Alternative: Ohne RViz für Performance
# ros2 run go2_my_nodes_py hole_detection_voxel_grid
