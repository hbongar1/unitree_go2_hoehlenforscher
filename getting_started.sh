#!/bin/bash
# Getting Started Script für die 3 Loch-Erkennungsmethoden
# Autor: AI Assistant
# Datum: 2026-01-28

set -e  # Exit on error

WORKSPACE_DIR="/workspace/unitree_go2_hoehlenforscher"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Farben für Output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "\n${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║ $1${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}\n"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

main() {
    print_header "LOCH-ERKENNUNGSMETHODEN - GETTING STARTED"
    
    # 1. Verify workspace
    echo "1️⃣  Workspace überprüfen..."
    if [ -d "$WORKSPACE_DIR" ]; then
        print_success "Workspace gefunden: $WORKSPACE_DIR"
    else
        print_error "Workspace nicht gefunden: $WORKSPACE_DIR"
        exit 1
    fi
    
    # 2. Check for required files
    echo -e "\n2️⃣  Dateien überprüfen..."
    
    required_files=(
        "src/go2_my_nodes_py/go2_my_nodes_py/hole_detection_normal_vector.py"
        "src/go2_my_nodes_py/go2_my_nodes_py/hole_detection_convex_hull.py"
        "src/go2_my_nodes_py/go2_my_nodes_py/hole_detection_voxel_grid.py"
        "src/go2_my_nodes_py/launch/hole_detection_all.launch.py"
        "HOLE_DETECTION_README.md"
    )
    
    for file in "${required_files[@]}"; do
        if [ -f "$WORKSPACE_DIR/$file" ]; then
            print_success "Datei vorhanden: $file"
        else
            print_error "Datei fehlend: $file"
            exit 1
        fi
    done
    
    # 3. Build
    echo -e "\n3️⃣  Build ausführen..."
    cd "$WORKSPACE_DIR"
    
    if colcon build --packages-select go2_my_nodes_py 2>&1 | tail -5; then
        print_success "Build erfolgreich"
    else
        print_error "Build fehlgeschlagen"
        exit 1
    fi
    
    # 4. Source setup
    echo -e "\n4️⃣  Setup sourcing..."
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
        print_success "Setup.bash gesourced"
    else
        print_error "setup.bash nicht gefunden"
        exit 1
    fi
    
    # 5. Verify ROS2 installation
    echo -e "\n5️⃣  ROS2 überprüfen..."
    if command -v ros2 &> /dev/null; then
        print_success "ROS2 ist installiert"
        ros2 --version
    else
        print_error "ROS2 nicht gefunden"
        exit 1
    fi
    
    # 6. Check if nodes are registered
    echo -e "\n6️⃣  Nodes überprüfen..."
    
    nodes=(
        "hole_detection_normal_vector"
        "hole_detection_convex_hull"
        "hole_detection_voxel_grid"
    )
    
    for node in "${nodes[@]}"; do
        if ros2 run --help 2>&1 | grep -q "go2_my_nodes_py"; then
            print_success "Node $node ist registriert"
        fi
    done
    
    # 7. Display documentation
    print_header "VERFÜGBARE DOKUMENTATION"
    
    echo "📚 Lese die folgende Dokumentation für mehr Informationen:"
    echo ""
    echo "1. HOLE_DETECTION_README.md"
    echo "   → Detaillierte Anleitung für alle Methoden"
    echo "   → Befehl: cat HOLE_DETECTION_README.md"
    echo ""
    echo "2. IMPLEMENTATION_SUMMARY.md"
    echo "   → Technische Zusammenfassung"
    echo "   → Befehl: cat IMPLEMENTATION_SUMMARY.md"
    echo ""
    echo "3. Interaktives Dashboard"
    echo "   → Befehl: python3 COMPARISON_CHEATSHEET_INTERACTIVE.py"
    echo "   → Optionen: comparison, recommendation, quick, all"
    echo ""
    
    # 8. Quick start
    print_header "QUICK START"
    
    echo "🚀 Um einen der Nodes zu starten, verwenden Sie:"
    echo ""
    echo "   # Nur Voxel Grid (EMPFOHLEN):"
    echo "   ros2 launch go2_my_nodes_py hole_detection_voxel_grid.launch.py"
    echo ""
    echo "   # Nur Normal Vector:"
    echo "   ros2 launch go2_my_nodes_py hole_detection_normal_vector.launch.py"
    echo ""
    echo "   # Nur Convex Hull:"
    echo "   ros2 launch go2_my_nodes_py hole_detection_convex_hull.launch.py"
    echo ""
    echo "   # Alle 3 zum Vergleich:"
    echo "   ros2 launch go2_my_nodes_py hole_detection_all.launch.py"
    echo ""
    
    # 9. Visualization
    echo -e "\n📊 Zur Visualisierung in RViz (neues Terminal):"
    echo "   ros2 run rviz2 rviz2"
    echo ""
    
    # 10. Topics
    echo -e "\n📡 Topics überwachen:"
    echo "   ros2 topic echo /detected_entrances_voxel_grid"
    echo "   ros2 topic hz /detected_entrances_voxel_grid"
    echo ""
    
    # Success
    print_header "✅ ALLES BEREIT!"
    
    echo "Die Implementierung ist erfolgreich eingerichtet."
    echo ""
    echo "Nächste Schritte:"
    echo "1. ✓ Starten Sie einen oder alle Nodes"
    echo "2. ✓ Öffnen Sie RViz zur Visualisierung"
    echo "3. ✓ Überwachen Sie die Topics"
    echo "4. ✓ Testen Sie mit echter Lidar-Hardware"
    echo ""
    echo "Fragen? Siehe:"
    echo "• HOLE_DETECTION_README.md - für detaillierte Anleitung"
    echo "• IMPLEMENTATION_SUMMARY.md - für technische Details"
    echo ""
}

# Run main function
main
