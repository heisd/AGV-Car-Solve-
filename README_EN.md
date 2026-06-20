# Multi-AGV Warehouse Simulation System

English | [简体中文](./README.md)

A Multi-AGV (Autonomous Mobile Robot) intelligent warehouse dispatching and collaboration simulation system based on ROS 2 Humble and Gazebo Classic 11. It integrates multi-robot path planning, narrow corridor Right-of-Way (RoW) lock reservation, YOLOv8-based vision target detection, and an interactive web monitoring dashboard.

## 🚀 System Features

1. **Dual Navigation Drivers**:
   - **Nav2 Industrial Navigation**: Features AMCL localization, Costmap dynamic obstacle avoidance, and Behavior Tree (BT) navigation.
   - **Ground Truth Fallback**: A lightweight simulator ground truth Proportional control tracker that drives the robot node-to-node directly to reduce CPU overhead.
2. **Corridor Right-of-Way (RoW) Resolution**:
   - Manages and avoids deadlocks when multiple AGVs attempt to navigate narrow single-lane corridors. Incorporates a reservation locking queue for mutual exclusion and cooperative waiting.
3. **AI Vision Perception**:
   - Performs real-time target recognition with YOLOv8 via simulated onboard cameras (`amr_vision`).
4. **Web Control Dashboard**:
   - Real-time HTML/JS status monitoring over Rosbridge. Renders maps, AGV batteries, coordinates, navigation path lines, and RoW locks. Supports one-key mission dispatch and manual driving.

## 📂 Workspace Structure

```text
.
├── src/
│   ├── amr_description/       # URDF robot models, Gazebo worlds, Nav2 launchers, and fleet_manager
│   ├── amr_vision/            # YOLOv8 target detection and camera nodes
│   └── amr_web/               # Rosbridge Web frontend files and launch configuration
├── doc/
│   ├── operations-guide.md                 # Setup, compilation, and launch commands
│   ├── navigation-modes-and-fallback.md    # Dual-driver navigation and status readiness
│   ├── right-of-way-design.md              # Corridor reservation design specification
│   └── mapping-guide.md                    # 2D lidar SLAM guide
└── README_EN.md               # This document
```

## 🛠️ Quick Start

For detailed step-by-step guidance, please check the [Operations Guide](file:///home/li/GazeboLib/doc/operations-guide.md).

### 1. Install Dependencies
Requires Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11.
```zsh
sudo apt update
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs ros-humble-rosbridge-server ros-humble-xacro python3-transforms3d
python3 -m pip install --user -U "transforms3d>=0.4.2"
```

### 2. Build Workspace
Compile in sequential execution mode to avoid compiler workspace parallel lock contention:
```zsh
cd ~/GazeboLib
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --executor sequential
```

### 3. Launch System
Run in your workspace (starts 2 AGVs + Nav2 Stack + Web panel + Gazebo GUI by default):
```zsh
source install/setup.zsh
ros2 launch amr_description warehouse_fleet_nav2.launch.py
```
- Launch lightweight ground truth tracking fallback mode:
  ```zsh
  ros2 launch amr_description warehouse_fleet.launch.py
  ```

### 4. Open Web Dashboard
Wait 30-60 seconds for the Nav2 framework to initialize, then navigate to:
```text
http://localhost:8080/
```

## 📖 Related Documents
For underlying engineering design and configurations:
- [Operations & Troubleshooting Manual (operations-guide.md)](file:///home/li/GazeboLib/doc/operations-guide.md)
- [Dual Navigation Modes & Fallback Tracker (navigation-modes-and-fallback.md)](file:///home/li/GazeboLib/doc/navigation-modes-and-fallback.md)
- [Corridor Right-of-Way Locking Scheme (right-of-way-design.md)](file:///home/li/GazeboLib/doc/right-of-way-design.md)
- [Warehouse 2D Mapping Guidelines (mapping-guide.md)](file:///home/li/GazeboLib/doc/mapping-guide.md)
