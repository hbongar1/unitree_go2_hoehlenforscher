#!/bin/bash

# RViz Konfiguration für Vergleich der drei Loch-Erkennungsmethoden

cat > /tmp/hole_detection_comparison.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
      Splitter Ratio: 0.5
    Tree Height: 363
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.588679
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Preferences:
  PromptSaveOnExit: true
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Cell Size: 1
      Class: rviz_common/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Line
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: 0
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_common/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: Lidar Cloud
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.01
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /utlidar/cloud_deskewed
      Unreliable: false
      Use Full RGB: true
      Value: true
    - Alpha: 0.7
      Class: rviz_common/PointCloud2
      Color: 0; 255; 0
      Color Transformer: RGB
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 255
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: Filtered Cloud (Normal Vector)
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.01
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /filtered_cloud_normal_vector
      Unreliable: false
      Use Full RGB: false
      Value: true
    - Alpha: 0.7
      Class: rviz_common/PointCloud2
      Color: 255; 165; 0
      Color Transformer: RGB
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 255
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: Filtered Cloud (Convex Hull)
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.01
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /filtered_cloud_convex_hull
      Unreliable: false
      Use Full RGB: false
      Value: true
    - Alpha: 0.7
      Class: rviz_common/PointCloud2
      Color: 0; 255; 255
      Color Transformer: RGB
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 255
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: Filtered Cloud (Voxel Grid)
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.01
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /filtered_cloud_voxel_grid
      Unreliable: false
      Use Full RGB: false
      Value: true
    - Class: rviz_common/MarkerArray
      Enabled: true
      Name: Cluster Markers (Normal Vector)
      Namespaces:
        cluster_markers_normal_vector: true
      Queue Size: 100
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /cluster_markers_normal_vector
      Unreliable: false
      Value: true
    - Class: rviz_common/MarkerArray
      Enabled: true
      Name: Cluster Markers (Convex Hull)
      Namespaces:
        cluster_markers_convex_hull: true
      Queue Size: 100
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /cluster_markers_convex_hull
      Unreliable: false
      Value: true
    - Class: rviz_common/MarkerArray
      Enabled: true
      Name: Cluster Markers (Voxel Grid)
      Namespaces:
        cluster_markers_voxel_grid: true
      Queue Size: 100
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /cluster_markers_voxel_grid
      Unreliable: false
      Value: true
    - Class: rviz_common/MarkerArray
      Enabled: true
      Name: Entrances (Normal Vector)
      Namespaces:
        entrance_markers_normal_vector: true
      Queue Size: 100
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /entrance_markers_normal_vector
      Unreliable: false
      Value: true
    - Class: rviz_common/MarkerArray
      Enabled: true
      Name: Entrances (Convex Hull)
      Namespaces:
        entrance_markers_convex_hull: true
      Queue Size: 100
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /entrance_markers_convex_hull
      Unreliable: false
      Value: true
    - Class: rviz_common/MarkerArray
      Enabled: true
      Name: Entrances (Voxel Grid)
      Namespaces:
        entrance_markers_voxel_grid: true
      Queue Size: 100
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /entrance_markers_voxel_grid
      Unreliable: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_common/Interact
      Hide Inactive Objects: true
    - Class: rviz_common/MoveCamera
    - Class: rviz_common/Select
    - Class: rviz_common/FocusCamera
    - Class: rviz_common/Measure
      Class Name: rviz_common/Measure
    - Class: rviz_ros_common/Navigation2D
      Class Name: rviz_ros_common/Navigation2D
    - Class: rviz_common/PublishPoint
      Single click publishing: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_common/TF
  Value: true
  Views:
    Current:
      Class: rviz_common/Orbit
      Distance: 5
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.785398
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398
    Saved: []
Window Geometry:
  Displays:
    collapsed: false
  Height: 1056
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd00000004000000000000000000000000fc0200000009fb0000001200530065006c0065006300740069006f006e00000000770000004d0000006700fffe0006fb0000001e0054006f006f006c002000500072006f0070006500720074006900650073000001f70000004d0000005e0fffffffb000000120043006f006e00740065007800740020004d006f006e000000017b0000009a0000000000000000fb0000000a0056006900650077007300000002bc000000c40000010f0fffffffb0000001200440069007300700006c0061007900730100000000000003f8000004650fffffffb0000000a0049006d0061006700650000000505000000c60000000000000000000005380000044600000001000000020000000100000002fc0000003b000000270100001600fffffffb0000001e0054006f006f006c002000500072006f0070006500720074006900650073010000004d0000004d0000000000000000fb0000001200530065006c0065006300740069006f006e0100000000ffffffff0000000000000000
Width: 1824
X: 0
Y: 0
EOF

echo "RViz Konfiguration erstellt: /tmp/hole_detection_comparison.rviz"
