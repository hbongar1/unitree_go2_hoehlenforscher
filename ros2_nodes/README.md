# ROS2 Entrance Detection System

This directory contains the ROS2 implementation of the entrance detection system for the Unitree GO2 robot.

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

## Quick Start

### 1. Build Custom Messages (One-time setup)

If you have a ROS2 workspace:

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

### 2. Run Nodes

#### Option A: Individual Nodes (for testing)

```bash
# Terminal 1: Depth Camera Node
python3 depth_camera_node.py

# Terminal 2: Verarbeitung Node  
python3 verarbeitung_node.py

# Terminal 3: Verhalten Node
python3 verhalten_node.py

# Terminal 4: Ausfuehrung Node (Simulation mode)
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
┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌─────────────┐   ┌──────────────┐
│depth_camera_ │   │unitree_lidar │   │verarbeitung  │   │ verhalten   │   │ ausfuehrung  │
│node (Camera) │──>│_node (LiDAR) │──>│_node (Fusion)│──>│_node (State)│──>│_node (Motor) │
└──────────────┘   └──────────────┘   └──────────────┘   └─────────────┘   └──────────────┘
   /sensor/              /perception/          Decision            /status/
   depth_data            entrance              Logic               motor
                         entrance_state
```

## Troubleshooting

### RealSense Not Found
```bash
# Check USB connection
lsusb | grep Intel

# Install drivers
sudo apt install ros-humble-librealsense2
```

### Unitree Connection Failed
```bash
# Check network interface name
ifconfig

# Update params.yaml with correct interface name
# Test ping to robot (if using Ethernet)
ping <robot-ip>
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
