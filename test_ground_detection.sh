#!/bin/bash
# Quick-Check Script für Bodenerkennung

echo "🔍 Teste Bodenerkennung..."
echo ""

cd "$(dirname "$0")"

# Source ROS2
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "⚠️  install/setup.bash nicht gefunden. Bitte erst bauen:"
    echo "   ./test_cardboard_entrance.sh"
    exit 1
fi

echo "Starte Node für 10 Sekunden zum Testen..."
echo "Achte auf LOG-Ausgaben:"
echo "  - '✓ Boden erkannt bei X.XXm'"
echo "  - '✓ YYYY Punkte über dem Boden'"
echo ""

timeout 10 ros2 run go2_my_nodes_py hole_detection_voxel_grid \
    --ros-args --params-file src/go2_my_nodes_py/config/cardboard_detection_params.yaml

echo ""
echo "Wenn 'sklearn nicht verfügbar' erscheint, installiere es:"
echo "  pip install scikit-learn"
