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

For detailed step-by-step guidance, please check the [Operations Guide](./doc/operations-guide.md).

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
git clone --branch WSLHumble https://github.com/heisd/AGV-Car-Solve-.git
cd AGV-Car-Solve-
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
- [Operations & Troubleshooting Manual (operations-guide.md)](./doc/operations-guide.md)
- [Dual Navigation Modes & Fallback Tracker (navigation-modes-and-fallback.md)](./doc/navigation-modes-and-fallback.md)
- [Corridor Right-of-Way Locking Scheme (right-of-way-design.md)](./doc/right-of-way-design.md)
- [Warehouse 2D Mapping Guidelines (mapping-guide.md)](./doc/mapping-guide.md)

## Navigation modes and their limits

| Aspect | Nav2 mode | Ground-truth follower |
| --- | --- | --- |
| Launch file | warehouse_fleet_nav2.launch.py | warehouse_fleet.launch.py |
| Navigation path | Localization, planning and Nav2 interfaces | Simulator pose and a lightweight follower |
| Intended use | Navigation integration and fleet experiments | Quick demonstrations and constrained machines |
| Obstacle handling | Inspect maps, costmaps and planner behavior | Forward slowing is not general obstacle rerouting |
| Dashboard readiness | Reports Nav2 readiness | May stay "not ready" because Nav2 is not running |

See the [navigation architecture guide](./doc/navigation-modes-and-fallback.md). Simulator ground truth is a convenience for experiments and does not validate a real-robot navigation stack.

## Launch configuration

The operations guide documents these common parameters:

| Parameter | Default | Purpose |
| --- | --- | --- |
| num_agvs | 2 | Fleet size |
| gui | true | Gazebo GUI |
| use_web | true | Web dashboard |
| world | Packaged warehouse | World file path |

```zsh
ros2 launch amr_description warehouse_fleet_nav2.launch.py --show-args
ros2 launch amr_description warehouse_fleet_nav2.launch.py num_agvs:=1 gui:=false use_web:=false
```

Start with one robot when diagnosing a problem, then restore the default two-robot setup. More robots increase simulation and navigation load; the repository does not provide a performance guarantee for every machine.

The examples use zsh. In Bash, load the matching `setup.bash` files instead of `setup.zsh`.

## First task

Open the dashboard, choose pickup and drop-off regions, and submit a transport task. Watch the fleet state, task transitions and displayed path. Documented region names include:

| Regions | Purpose |
| --- | --- |
| pickup_A / pickup_B | Pickup |
| dropoff_A / dropoff_B | Drop-off |
| charger_1 | Charging |

The operations guide allows manual navigation for an IDLE robot; a robot executing a transport task should not be treated as idle. Use current configuration files for exact region coordinates and interface details.

## Troubleshooting by layer

```zsh
ros2 node list
ros2 topic list -t
ros2 topic echo /fleet/state --field data --once
ros2 topic echo /agv1/plan --once
```

| Symptom | Check |
| --- | --- |
| No robot in Gazebo | World loading, entity spawning and launch logs |
| Dashboard unavailable | Web server, port 8080 and local connectivity |
| Dashboard has no fleet data | rosbridge, ROS nodes, topics and browser connection |
| Nav2 not ready | Clock, TF, localization, lifecycle and resource pressure |
| Robot waits in a corridor | Distinguish right-of-way waiting from planning failure |
| Goal unreachable | Map, obstacles, coordinates and planning result |

Stop the launch terminal with `Ctrl+C` and confirm its processes exit before restarting. Leftover nodes can cause duplicate TF, conflicting state or occupied ports.

## Detailed guides

- [Operations and troubleshooting](./doc/operations-guide.md)
- [Navigation and fallback](./doc/navigation-modes-and-fallback.md)
- [Corridor right-of-way design](./doc/right-of-way-design.md)
- [Mapping](./doc/mapping-guide.md)

This is a warehouse simulation and integration project. The documentation does not establish real-hardware readiness or freedom from deadlocks in every possible scenario.
