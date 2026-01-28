#!/usr/bin/env python3
"""
Vergleichs-Cheat-Sheet für die drei Loch-Erkennungsmethoden

Dieses Skript zeigt einen direkten Vergleich der drei Implementierungen
"""

comparison_data = {
    "Normal Vector Analysis": {
        "Farbschema": {
            "Cluster": "Blau (0.5, 0.5, 1.0)",
            "Eingänge": "Grün (0.0, 1.0, 0.0)",
        },
        "Hauptparameter": {
            "normal_search_radius": "0.1m - Radius für Nachbarschafts-Suche",
            "normal_variance_threshold": "0.15 - Schwellwert für Loch-Erkennung",
            "normal_window_size": "10 - Punkte pro Analyse-Fenster",
        },
        "Vorteile": [
            "✅ Sehr gute Genauigkeit bei Kanten-Erkennung",
            "✅ Erfasst auch kleine Löcher",
            "✅ Mathematisch fundiert (PCA)",
        ],
        "Nachteile": [
            "❌ Hohe CPU-Last (Normal-Berechnung teuer)",
            "❌ Empfindlich gegen Lidar-Rauschen",
            "❌ Komplexe Parameter-Tuning",
            "❌ Braucht scipy (KDTree)",
        ],
        "Komplexität": "⭐⭐⭐⭐ (Hoch)",
        "Geschwindigkeit": "⭐⭐⭐ (Mittel)",
        "Best für": "Hohe Genauigkeit wenn CPU verfügbar",
        "ROS Topics": {
            "Input": "/utlidar/cloud_deskewed",
            "Output": "/detected_entrances_normal_vector",
            "Marker": "/entrance_markers_normal_vector",
        }
    },
    
    "Convex Hull Analysis": {
        "Farbschema": {
            "Cluster": "Orange (1.0, 0.5, 0.0)",
            "Eingänge": "Orange-Braun (1.0, 0.647, 0.0)",
        },
        "Hauptparameter": {
            "hole_volume_ratio_threshold": "0.15 - Anteil fehlender Volumen",
            "min_interior_points": "50 - Minimum innere Punkte",
        },
        "Vorteile": [
            "✅ Sehr schnell (O(n log n))",
            "✅ Robust gegen Rauschen",
            "✅ Einfaches Konzept",
            "✅ Standard-Geometrie-Algorithmus",
        ],
        "Nachteile": [
            "❌ Schlecht bei kleine Löchern",
            "❌ Überschätzt Löcher bei unregelmäßigen Formen",
            "❌ Braucht scipy für ConvexHull",
        ],
        "Komplexität": "⭐⭐ (Niedrig)",
        "Geschwindigkeit": "⭐⭐⭐⭐ (Schnell)",
        "Best für": "Robustheit & große Löcher",
        "ROS Topics": {
            "Input": "/utlidar/cloud_deskewed",
            "Output": "/detected_entrances_convex_hull",
            "Marker": "/entrance_markers_convex_hull",
        }
    },
    
    "Voxel Grid Analysis": {
        "Farbschema": {
            "Cluster": "Cyan (0.0, 1.0, 1.0)",
            "Eingänge": "Grün (0.0, 1.0, 0.0)",
        },
        "Hauptparameter": {
            "voxel_size": "0.05m - Größe eines Voxels",
            "gaussian_sigma": "1.0 - Glättungs-Parameter",
            "gradient_percentile": "75 - Top % für Kanten",
            "low_density_percentile": "25 - Unten % für Löcher",
        },
        "Vorteile": [
            "✅ SEHR SCHNELL (O(n))",
            "✅ Adaptive Voxel-Größe möglich",
            "✅ Robust gegen Rauschen (Gaussian Filter)",
            "✅ Erfasst alle Loch-Größen",
            "✅ Beste Performance/Genauigkeit Balance",
            "✅ GPU-beschleunigung möglich",
        ],
        "Nachteile": [
            "❌ Voxel-Größe kritischer Parameter",
        ],
        "Komplexität": "⭐⭐ (Niedrig)",
        "Geschwindigkeit": "⭐⭐⭐⭐⭐ (SEHR schnell)",
        "Best für": "Echtzeit-Robotik, embedded systems",
        "ROS Topics": {
            "Input": "/utlidar/cloud_deskewed",
            "Output": "/detected_entrances_voxel_grid",
            "Marker": "/entrance_markers_voxel_grid",
        }
    }
}

print("=" * 80)
print("LOCH-ERKENNUNGSMETHODEN VERGLEICH")
print("=" * 80)
print()

for method_name, data in comparison_data.items():
    print(f"\n{'='*80}")
    print(f"🔍 {method_name.upper()}")
    print(f"{'='*80}")
    
    print(f"\n📊 Farbcodierung in RViz:")
    for key, val in data["Farbschema"].items():
        print(f"   {key}: {val}")
    
    print(f"\n⚙️  Hauptparameter:")
    for param, desc in data["Hauptparameter"].items():
        print(f"   • {param}")
        print(f"     → {desc}")
    
    print(f"\n✅ Vorteile:")
    for adv in data["Vorteile"]:
        print(f"   {adv}")
    
    print(f"\n❌ Nachteile:")
    for dis in data["Nachteile"]:
        print(f"   {dis}")
    
    print(f"\n📈 Metriken:")
    print(f"   Komplexität: {data['Komplexität']}")
    print(f"   Geschwindigkeit: {data['Geschwindigkeit']}")
    print(f"   Best für: {data['Best für']}")
    
    print(f"\n📡 ROS2 Topics:")
    for topic_type, topic_name in data["ROS Topics"].items():
        print(f"   {topic_type}: {topic_name}")


print("\n" + "=" * 80)
print("EMPFEHLUNGSMATRIX")
print("=" * 80)

recommendations = """
┌─────────────────────────────────────────────────────────────────┐
│ Wähle VOXEL GRID wenn...                                        │
├─────────────────────────────────────────────────────────────────┤
│ ✓ Du schnelle Echtzeit-Verarbeitung brauchst                    │
│ ✓ Du auf embedded Hardware laufen möchtest                      │
│ ✓ Du alle Loch-Größen erfassen möchtest                         │
│ ✓ Du Lidar-Rauschen tolerieren möchtest                         │
│ ✓ Du Parameter leicht tunen möchtest                            │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ Wähle NORMAL VECTOR wenn...                                     │
├─────────────────────────────────────────────────────────────────┤
│ ✓ Du maximale Genauigkeit brauchst                              │
│ ✓ Du CPU-Power zur Verfügung hast                               │
│ ✓ Du kleine/irreguläre Löcher erkennen musst                    │
│ ✓ Du Kanten-Erkennung brauchst                                  │
│ ✗ Nicht geeignet für Real-Time auf Robotern                     │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ Wähle CONVEX HULL wenn...                                       │
├─────────────────────────────────────────────────────────────────┤
│ ✓ Du große Löcher erkennen möchtest                             │
│ ✓ Du maximale Robustheit gegen Rauschen brauchst                │
│ ✓ Du einfache Parameter-Logik magst                             │
│ ✗ Nicht ideal für kleine Löcher                                 │
│ ✗ Überschätzt bei unregelmäßigen Formen                         │
└─────────────────────────────────────────────────────────────────┘

GESAMTSIEGER: VOXEL GRID 🏆
└─ Beste Balance aus Speed, Genauigkeit, Robustheit
"""
print(recommendations)

print("\n" + "=" * 80)
print("HÄUFIGE PARAMETER-ANPASSUNGEN")
print("=" * 80)

tuning = """
PROBLEM: "Zu viele False Positives"
→ Erhöhe confidence_threshold (2 → 3 oder 4)
→ Erhöhe height_threshold oder width_threshold
→ Reduziere cluster_distance (0.5 → 0.3)

PROBLEM: "Zu langsam"
→ Nutze VOXEL GRID statt Normal Vector
→ Erhöhe process_every_n_frames (5 → 10)
→ Erhöhe voxel_size (0.05 → 0.1)

PROBLEM: "Misst kleine Löcher nicht"
→ Nutze NORMAL VECTOR statt Convex Hull
→ Reduziere voxel_size (0.05 → 0.03)
→ Erhöhe frame_buffer_size (20 → 30)

PROBLEM: "Zu empfindlich gegen Rauschen"
→ Nutze VOXEL GRID oder CONVEX HULL
→ Erhöhe gaussian_sigma (1.0 → 2.0)
→ Erhöhe confidence_threshold

PROBLEM: "Verpasst echte Eingänge"
→ Reduziere Schwellwerte
→ Erhöhe frame_buffer_size
→ Nutze combination mehrerer Methoden (voting)
"""
print(tuning)

print("\n" + "=" * 80)
print("HYBRID-ANSATZ (Kombination)")
print("=" * 80)

hybrid = """
Für maximale Robustheit: Kombiniere alle 3 Methoden!

Abstimmungs-Strategie:
1. Starte alle 3 Nodes
2. Kombiniere Ergebnisse: "Eingang nur wenn ≥2 Methoden erkennen"
3. Gewichte nach Konfidenz

ROS-Kombinator-Node (pseudo-code):
────────────────────────────────────────────────
def combine_entrances():
    results = {
        'normal_vector': listen('/detected_entrances_normal_vector'),
        'convex_hull': listen('/detected_entrances_convex_hull'),
        'voxel_grid': listen('/detected_entrances_voxel_grid'),
    }
    
    for entrance in results['normal_vector']:
        # Suche Matches in anderen Methoden
        match_count = count_matches(entrance, results)
        
        if match_count >= 2:  # Mindestens 2 Methoden
            publish_final_entrance(entrance)
────────────────────────────────────────────────

Vorteile:
✓ Eliminiert False Positives
✓ Verbessert Robustheit
✓ Nutzt Stärken aller 3 Methoden

Nachteil:
✗ 3x CPU-Last
✗ Könnte echte Eingänge missen (if voting too strict)

Empfehlung: Nur für hochkritische Anwendungen
"""
print(hybrid)

print("\n" + "=" * 80)
print("MONITORING & DEBUGGING")
print("=" * 80)

monitoring = """
# Live-Vergleich in Terminal:
ros2 topic echo /detected_entrances_normal_vector
ros2 topic echo /detected_entrances_convex_hull
ros2 topic echo /detected_entrances_voxel_grid

# Frequency-Vergleich:
ros2 topic hz /detected_entrances_voxel_grid

# In RViz: Visualisiere alle 3 gleichzeitig!
- Aktiviere alle "Entrance Markers" und "Filtered Cloud" Displays
- Vergleiche visuell die Erkennungen
- Beobachte Unterschiede in schwierigen Szenen

# Logfile für Analyse:
ros2 launch go2_my_nodes_py hole_detection_all.launch.py > comparison.log 2>&1
grep -i "loch erkannt\|eingang" comparison.log
"""
print(monitoring)

print("\n" + "=" * 80)
print("QUICK REFERENCE - Command Cheat Sheet")
print("=" * 80)

commands = """
# BUILD
colcon build --packages-select go2_my_nodes_py
source install/setup.bash

# SINGLE METHOD
ros2 launch go2_my_nodes_py hole_detection_voxel_grid.launch.py

# ALL METHODS (Comparison)
ros2 launch go2_my_nodes_py hole_detection_all.launch.py

# VISUALIZE
ros2 run rviz2 rviz2

# MONITOR TOPICS
ros2 topic list | grep detection
ros2 topic echo /detected_entrances_voxel_grid

# RECORD FOR TESTING
ros2 bag record -o test_data /utlidar/cloud_deskewed

# REPLAY RECORDED DATA
ros2 bag play test_data.mcap

# NODE INFO
ros2 node list | grep hole
ros2 node info /hole_detection_voxel_grid_node
"""
print(commands)

print("\n" + "=" * 80)
print("✅ Implementation Complete - Ready to Test!")
print("=" * 80)
