#!/usr/bin/env python3
"""
Interaktives Vergleichs-Dashboard für die 3 Loch-Erkennungsmethoden.
Zeigt Metriken, Performance und Live-Vergleich.
"""

import sys
import time
import subprocess
from dataclasses import dataclass
from typing import Dict, List


@dataclass
class MethodMetrics:
    name: str
    color: str
    cpu_estimate: str
    speed: str
    accuracy: str
    noise_robust: str
    best_for: str
    topics: Dict[str, str]


def print_header(text: str, char: str = "="):
    """Druckt einen formatierten Header"""
    width = 80
    padding = (width - len(text) - 2) // 2
    print(f"\n{char * width}")
    print(f"{char} {text:^{width-4}} {char}")
    print(f"{char * width}\n")


def print_method_card(method: MethodMetrics):
    """Druckt eine formatierte Methoden-Karte"""
    print(f"┌{'─' * 76}┐")
    print(f"│ {method.name:^74} │")
    print(f"├{'─' * 76}┤")
    print(f"│ Farbe: {method.color:70} │")
    print(f"│ CPU: {method.cpu_estimate:71} │")
    print(f"│ Geschwindigkeit: {method.speed:58} │")
    print(f"│ Genauigkeit: {method.accuracy:63} │")
    print(f"│ Rausch-Robustheit: {method.noise_robust:56} │")
    print(f"│ Best für: {method.best_for:67} │")
    print(f"└{'─' * 76}┘\n")


def compare_methods():
    """Zeigt Vergleichstabelle"""
    methods = [
        MethodMetrics(
            name="Normal Vector Analysis",
            color="Blau/Grün",
            cpu_estimate="⭐⭐⭐ (Hoch)",
            speed="⭐⭐⭐ (Mittel)",
            accuracy="⭐⭐⭐⭐ (Sehr gut)",
            noise_robust="⭐⭐ (Gering)",
            best_for="Maximale Genauigkeit",
            topics={
                "entrance": "/detected_entrances_normal_vector",
                "markers": "/entrance_markers_normal_vector",
                "cluster": "/cluster_markers_normal_vector",
                "cloud": "/filtered_cloud_normal_vector"
            }
        ),
        MethodMetrics(
            name="Convex Hull Analysis",
            color="Orange/Braun",
            cpu_estimate="⭐⭐⭐⭐ (Sehr schnell)",
            speed="⭐⭐⭐⭐ (Schnell)",
            accuracy="⭐⭐⭐ (Gut)",
            noise_robust="⭐⭐⭐⭐ (Sehr gut)",
            best_for="Große Löcher & Robustheit",
            topics={
                "entrance": "/detected_entrances_convex_hull",
                "markers": "/entrance_markers_convex_hull",
                "cluster": "/cluster_markers_convex_hull",
                "cloud": "/filtered_cloud_convex_hull"
            }
        ),
        MethodMetrics(
            name="Voxel Grid Analysis 🏆",
            color="Cyan/Grün",
            cpu_estimate="⭐⭐⭐⭐⭐ (SEHR schnell)",
            speed="⭐⭐⭐⭐⭐ (SEHR Schnell)",
            accuracy="⭐⭐⭐⭐ (Sehr gut)",
            noise_robust="⭐⭐⭐⭐ (Sehr gut)",
            best_for="Echtzeit-Robotik (EMPFOHLEN)",
            topics={
                "entrance": "/detected_entrances_voxel_grid",
                "markers": "/entrance_markers_voxel_grid",
                "cluster": "/cluster_markers_voxel_grid",
                "cloud": "/filtered_cloud_voxel_grid"
            }
        ),
    ]
    
    return methods


def print_comparison_table():
    """Druckt vergleichende Tabelle"""
    print_header("DETAILLIERTER VERGLEICH")
    
    methods = compare_methods()
    for method in methods:
        print_method_card(method)


def print_recommendation():
    """Druckt Empfehlung"""
    print_header("EMPFEHLUNG FÜR DIESES PROJEKT", "►")
    
    rec = """
╔════════════════════════════════════════════════════════════════════════════╗
║                         🏆 VOXEL GRID METHODE 🏆                          ║
╠════════════════════════════════════════════════════════════════════════════╣
║                                                                            ║
║  Gründe für diese Empfehlung:                                            ║
║  ✓ BESTE Performance (⭐⭐⭐⭐⭐)                                             ║
║  ✓ SEHR gute Genauigkeit (⭐⭐⭐⭐)                                          ║
║  ✓ Robust gegen Lidar-Rauschen                                           ║
║  ✓ Einfache Parameter (voxel_size hauptsächlich)                         ║
║  ✓ Skalierbar (kann auf GPU laufen)                                      ║
║  ✓ Für Unitree Go2 Roboter ideal                                         ║
║                                                                            ║
║  Ideale Settings für Ihre Use-Case:                                       ║
║  • voxel_size: 0.05m (adaptiv zu Punkt-Dichte)                           ║
║  • gaussian_sigma: 1.0 (gute Glättung)                                   ║
║  • gradient_percentile: 75 (Top 25% Kanten)                              ║
║  • confidence_threshold: 2 (stabil nach 2 Frames)                        ║
║                                                                            ║
║  Erwartete Performance:                                                   ║
║  • Latenzen: 50-100ms (bei 20Hz Lidar)                                   ║
║  • CPU: 15-25% auf Embedded (ggf. weniger)                               ║
║  • Memory: ~50-100MB                                                      ║
║                                                                            ║
╚════════════════════════════════════════════════════════════════════════════╝
    """
    print(rec)


def print_quick_start():
    """Druckt Quick-Start Anleitung"""
    print_header("QUICK START GUIDE")
    
    guide = """
1️⃣  BUILD
    $ cd /workspace/unitree_go2_hoehlenforscher
    $ colcon build --packages-select go2_my_nodes_py
    $ source install/setup.bash

2️⃣  STARTEN - Option A: NUR Voxel Grid (EMPFOHLEN)
    $ ros2 launch go2_my_nodes_py hole_detection_voxel_grid.launch.py

2️⃣  STARTEN - Option B: Alle 3 Methoden zum Vergleich
    $ ros2 launch go2_my_nodes_py hole_detection_all.launch.py

3️⃣  VISUALISIERUNG (neues Terminal)
    $ ros2 run rviz2 rviz2
    → Wähle Datei → "Open Config"
    → /workspace/entrance_detection.rviz

4️⃣  MONITORING
    $ ros2 topic echo /detected_entrances_voxel_grid
    oder
    $ ros2 topic hz /detected_entrances_voxel_grid

5️⃣  PARAMETER TUNEN (falls nötig)
    • Normal Vector: normal_variance_threshold ändern
    • Convex Hull: hole_volume_ratio_threshold ändern
    • Voxel Grid: voxel_size ändern (0.05 → 0.03 für Genauigkeit)
    """
    print(guide)


def print_topics_reference():
    """Druckt Topics-Referenz"""
    print_header("ROS2 TOPICS REFERENZ")
    
    methods = compare_methods()
    
    for method in methods:
        print(f"\n📡 {method.name}")
        print(f"{'─' * 76}")
        for topic_type, topic_name in method.topics.items():
            symbol = "→" if topic_type == "entrance" else "  "
            print(f"  {symbol} {topic_type:12}: {topic_name}")


def print_file_locations():
    """Zeigt Datei-Orte"""
    print_header("WICHTIGE DATEIEN")
    
    files = {
        "Normal Vector Node": "src/go2_my_nodes_py/go2_my_nodes_py/hole_detection_normal_vector.py",
        "Convex Hull Node": "src/go2_my_nodes_py/go2_my_nodes_py/hole_detection_convex_hull.py",
        "Voxel Grid Node": "src/go2_my_nodes_py/go2_my_nodes_py/hole_detection_voxel_grid.py",
        "Launch (All)": "src/go2_my_nodes_py/launch/hole_detection_all.launch.py",
        "Launch (Voxel)": "src/go2_my_nodes_py/launch/hole_detection_voxel_grid.launch.py",
        "Dokumentation": "HOLE_DETECTION_README.md",
        "Vergleichs-Cheat": "COMPARISON_CHEATSHEET.py",
    }
    
    for name, path in files.items():
        print(f"  📄 {name:25} → {path}")


def print_performance_guide():
    """Druckt Performance-Guide"""
    print_header("PERFORMANCE OPTIMIZATION GUIDE")
    
    guide = """
├─ FÜR SCHNELLE ECHTZEIT (Robotik):
│  └─ Nutze: VOXEL GRID
│     • voxel_size: 0.05-0.1m
│     • process_every_n_frames: 5-10
│     • frame_buffer_size: 10-15
│
├─ FÜR HÖCHSTE GENAUIGKEIT (Mapping):
│  └─ Nutze: NORMAL VECTOR
│     • normal_search_radius: 0.05-0.15m
│     • normal_window_size: 10-20
│     • process_every_n_frames: 2-3
│
├─ FÜR ROBUSTHEIT GEGEN RAUSCHEN:
│  └─ Nutze: CONVEX HULL oder VOXEL GRID
│     • hole_volume_ratio_threshold: 0.2
│     • gaussian_sigma: 1.5-2.0
│     • confidence_threshold: 3+
│
└─ HYBRID-ANSATZ (Beste Robustheit):
   └─ Starte alle 3 + kombiniere Ergebnisse
      • Eingang nur wenn ≥2 Methoden erkennen
      • Gewichte nach Konfidenz
      • CPU: 3x höher aber hochgradig robust

BENCHMARKS (auf Unitree Go2 geschätzt):
┌──────────────────┬───────┬────────┬──────────┐
│ Methode          │ CPU % │ Memory │ Latenz   │
├──────────────────┼───────┼────────┼──────────┤
│ Normal Vector    │ 25-35 │ 80MB   │ 100-150ms│
│ Convex Hull      │ 10-15 │ 60MB   │  50-100ms│
│ Voxel Grid       │  8-12 │ 70MB   │  30-50ms │ ✓
│ Alle 3 (Hybrid)  │ 45-60 │ 200MB  │ 100-150ms│
└──────────────────┴───────┴────────┴──────────┘
    """
    print(guide)


def print_testing_scenarios():
    """Druckt Test-Szenarien"""
    print_header("TEST-SZENARIEN FÜR VERGLEICH")
    
    scenarios = """
Teste die Methoden mit diesen Szenarien:

1️⃣  OFFENE TÜR / REALISTISCHES LOCH
   ├─ Größe: ~1m x 2m
   ├─ Expected: Alle 3 sollten gut erkennen
   └─ Erwartung: ✓✓✓ (Normal), ✓✓✓ (Hull), ✓✓✓ (Voxel)

2️⃣  KLEINE RISSE / DICHTE-ANOMALIEN
   ├─ Größe: ~0.2-0.5m x 1m
   ├─ Expected: Normal Vector & Voxel Grid besser
   └─ Erwartung: ✓✓✓ (Normal), ✓ (Hull), ✓✓✓ (Voxel)

3️⃣  STARKES RAUSCHEN (Regenwetter)
   ├─ Noise: >10% of measurements
   ├─ Expected: Convex Hull & Voxel Grid robuster
   └─ Erwartung: ✓ (Normal), ✓✓✓ (Hull), ✓✓✓ (Voxel)

4️⃣  MEHRERE TÜREN IM SICHTBEREICH
   ├─ Anzahl: 2-3 Eingänge
   ├─ Expected: Alle sollten separaten erkennen
   └─ Erwartung: ✓✓ (Normal), ✓✓ (Hull), ✓✓✓ (Voxel)

5️⃣  PERFORMANCE-TEST (20Hz Lidar)
   ├─ Duration: 1 Minute kontinuierlich
   ├─ Metric: CPU-Last, Memory, Latenz
   └─ Winner: ⭐ Voxel Grid

EMPFOHLENE TEST-REIHENFOLGE:
  1. Starte Voxel Grid (schneller)
  2. Wenn nicht zufrieden → Convex Hull
  3. Für Maximum Accuracy → Normal Vector
  4. Finale Tests mit Hybrid-Ansatz
    """
    print(scenarios)


def main():
    """Hauptprogramm"""
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
    else:
        command = "all"
    
    if command in ["comparison", "vergleich", "compare"]:
        print_comparison_table()
    elif command in ["recommendation", "empfehlung", "rec"]:
        print_recommendation()
    elif command in ["quick", "quickstart"]:
        print_quick_start()
    elif command in ["topics", "ros"]:
        print_topics_reference()
    elif command in ["files"]:
        print_file_locations()
    elif command in ["performance", "perf"]:
        print_performance_guide()
    elif command in ["scenarios", "tests", "test"]:
        print_testing_scenarios()
    elif command in ["help", "-h", "--help"]:
        print_help()
    elif command == "all":
        print_comparison_table()
        print_recommendation()
        print_quick_start()
        print_topics_reference()
        print_file_locations()
        print_performance_guide()
        print_testing_scenarios()
    else:
        print(f"Unbekannter Befehl: {command}")
        print_help()


def print_help():
    """Druckt Hilfe"""
    print_header("HILFE - VERWENDUNG")
    
    help_text = """
Verwendung: python3 COMPARISON_CHEATSHEET.py [BEFEHL]

Verfügbare Befehle:
  comparison    - Detaillierter Vergleich der Methoden
  recommendation - Empfehlung für dieses Projekt
  quick         - Quick Start Anleitung
  topics        - ROS2 Topics Referenz
  files         - Wichtige Datei-Orte
  performance   - Performance-Optimierungsguide
  scenarios     - Test-Szenarien für Vergleich
  all           - Alles anzeigen (default)
  help          - Diese Hilfe

Beispiele:
  python3 COMPARISON_CHEATSHEET.py comparison
  python3 COMPARISON_CHEATSHEET.py quick
  python3 COMPARISON_CHEATSHEET.py performance
    """
    print(help_text)


if __name__ == "__main__":
    main()
