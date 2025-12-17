# ROS2 Entrance Detection System

This directory contains the ROS2 implementation of the entrance detection system for the Unitree GO2 robot.

**Uses the official [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) SDK for robot control and sensor integration.**

## Structure

```
ros2_nodes/
├── entrance_detection_msgs/    # Custom ROS2 messages
│   ├── msg/
│   │   ├── DepthData.msg
│   │   ├── EntranceEvent.msg
│   │   ├── EntranceState.msg
│   │   ├── MotorStatus.msg
│   │   └── LidarGap.msg
│   ├── CMakeLists.txt
│   └── package.xml
│
├── depth_camera_node.py       # RealSense D435 depth camera
├── unitree_lidar_node.py      # Unitree 4D LiDAR L1
├── verarbeitung_node.py       # Entrance detection + sensor fusion
├── verhalten_node.py          # State machine / decision logic
├── ausfuehrung_node.py        # Unitree GO2 motor control
│
├── launch/
│   └── entrance_detection.launch.py
│
└── config/
    └── params.yaml
```

## Prerequisites

1. **ROS2 Foxy** installed (Ubuntu 20.04)
2. **unitree_ros2 SDK** installed (see [SETUP.md](../SETUP.md))
3. **RealSense SDK** for depth camera

## Quick Start

### 1. Setup unitree_ros2 SDK (One-time)

```bash
# Install unitree_ros2 (see SETUP.md for full instructions)
cd ~/ros2_ws/src
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Build Custom Messages (One-time setup)

```bash
# Copy entrance_detection_msgs to your ROS2 workspace
cp -r entrance_detection_msgs ~/ros2_ws/src/

# Build messages
cd ~/ros2_ws
colcon build --packages-select entrance_detection_msgs
source install/setup.bash

# Verify messages
ros2 interface list | grep entrance_detection_msgs
```

### 3. Run Nodes

**Note:** The `unitree_ros2` nodes for LiDAR and robot state run automatically when connected to the GO2.

#### Option A: Individual Nodes (for testing)

```bash
# Terminal 1: Depth Camera Node
python3 depth_camera_node.py

# Terminal 2: Unitree LiDAR Processing Node
python3 unitree_lidar_node.py

# Terminal 3: Verarbeitung Node (Sensor Fusion)
python3 verarbeitung_node.py

# Terminal 4: Verhalten Node (Decision Logic)
python3 verhalten_node.py

# Terminal 5: Ausfuehrung Node (Movement Control)
python3 ausfuehrung_node.py
```

#### Option B: Launch All Nodes

```bash
# Simulation mode (no robot)
ros2 launch launch/entrance_detection.launch.py simulation_mode:=true

# With real Unitree GO2 robot
ros2 launch launch/entrance_detection.launch.py simulation_mode:=false network_interface:=enp129s0
```

### 3. Monitor Topics

```bash
# List all topics
ros2 topic list

# Echo sensor data
ros2 topic echo /sensor/depth_image
ros2 topic echo /sensor/color_image

# Monitor entrance detection (after custom messages built)
# ros2 topic echo /perception/entrance
# ros2 topic echo /perception/entrance_state

# Check motor status
# ros2 topic echo /status/motor
```

### 4. Visualize with RViz2

```bash
rviz2
# Add displays for:
# - /sensor/color_image (Image)
# - /sensor/depth_image (Image)
```

## Configuration

Edit `config/params.yaml` to adjust:
- Camera resolution and FPS
- Entrance detection thresholds
- Robot height settings
- Unitree network interface
- Simulation mode

## Architecture

```
unitree_ros2 SDK                     Custom Nodes
┌──────────────┐                  ┌──────────────┐
│   /utlidar/  │                  │depth_camera_ │
│    cloud     │───┐              │node (Camera) │
└──────────────┘   │              └──────────────┘
                   │                     │
                   ▼                     ▼
            ┌──────────────┐      ┌──────────────┐      ┌─────────────┐      ┌──────────────┐
            │unitree_lidar │      │verarbeitung  │      │ verhalten   │      │ ausfuehrung  │
            │_node (Proc.) │─────>│_node (Fusion)│─────>│_node (State)│─────>│_node (Move)  │
            └──────────────┘      └──────────────┘      └─────────────┘      └──────────────┘
                                        │                                            │
                                        ▼                                            ▼
                                  /perception/                                  /cmd_vel
                                   entrance                                  (unitree_ros2)
```

### Topic Flow:
- **Inputs:** `/utlidar/cloud` (from unitree_ros2), `/sensor/depth_image` (RealSense)
- **Processing:** Gap detection, sensor fusion, decision logic
- **Output:** `/cmd_vel` (movement commands to unitree_ros2)

## Troubleshooting

### RealSense Not Found
```bash
# Check USB connection
lsusb | grep Intel

# Install drivers
sudo apt install ros-foxy-librealsense2
```

### Unitree Topics Not Visible
```bash
# Ensure unitree_ros2 is running
ros2 topic list | grep utlidar

# Check network connection to GO2
ping 192.168.123.15

# Restart unitree_ros2 nodes if needed
# (Usually auto-start when GO2 is on)
```

### Custom Messages Not Found
```bash
# Ensure messages are built
cd ~/ros2_ws
colcon build --packages-select entrance_detection_msgs
source install/setup.bash
```

## Next Steps

1. Build custom messages in ROS2 workspace
2. Test depth_camera_node with real RealSense camera
3. Test verarbeitung_node entrance detection
4. Integrate with Unitree GO2 for end-to-end testing
