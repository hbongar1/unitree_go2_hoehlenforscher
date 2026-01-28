#!/bin/bash
# Installation Script für Wall and Hole Detection Node
# Für ROS2 Humble auf Ubuntu 22.04

echo "================================================"
echo "Installing Wall and Hole Detection Dependencies"
echo "================================================"

# Update System
echo "Updating system packages..."
sudo apt update
sudo apt upgrade -y

# ROS2 Humble essentials (falls nicht bereits installiert)
echo "Installing ROS2 Humble development tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep

# Python Dependencies für die Wall and Hole Detection
echo "Installing Python dependencies..."
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-scipy

# Open3D für Point Cloud Verarbeitung (Hauptabhängigkeit!)
echo "Installing Open3D..."
pip3 install open3d

# ROS2 Python Packages
echo "Installing ROS2 Python packages..."
sudo apt install -y \
    python3-sensor-msgs \
    python3-geometry-msgs \
    python3-std-msgs \
    python3-visualization-msgs \
    ros-humble-sensor-msgs-py

# Optional: Visualisierungswerkzeuge
echo "Installing visualization tools..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-common

echo "================================================"
echo "Installation complete!"
echo "================================================"
echo ""
echo "Nächste Schritte:"
echo "1. Baue das ROS2 Package: colcon build"
echo "2. Source dein Workspace: source install/setup.bash"
echo "3. Starte den Node: ros2 run entrance_detection wall_hole_detection_node.py"
echo "   oder mit Launch:"
echo "   ros2 launch entrance_detection wall_hole_detection.launch.py"
