#!/bin/bash
# Komplettes Demo-Script: Voxel Grid + Bodenerkennung + RViz

# 1. Sicherstellen, dass scikit-learn installiert ist (für Bodenerkennung)
if ! python3 -c "import sklearn" &> /dev/null; then
    echo "⚠️  'scikit-learn' fehlt! Installiere es für die Bodenerkennung..."
    pip install scikit-learn
    echo "✅ Installation fertig."
fi

# 2. Workspace bauen (um sicherzugehen, dass alles aktuell ist)
echo "🔨 Baue Workspace..."
cd "$(dirname "$0")"
colcon build --packages-select go2_my_nodes_py --symlink-install

# 3. Source Setup
source install/setup.bash

echo "🚀 Starte Entrance Detection Demo..."
echo "   - Voxel Grid Methode"
echo "   - Bodenerkennung (RANSAC) AKTIV"
echo "   - Parameter aus: config/cardboard_detection_params.yaml"
echo "   - Visualisierung in RViz2"
echo ""

# 4. Launch file starten (Läd Voxel Node + Params + RViz)
ros2 launch go2_my_nodes_py cardboard_test.launch.py
