# 多 AGV 仓库仿真系统

[English Version](./README_EN.md) | 简体中文

基于 ROS 2 Humble + Gazebo Classic 11 + Navigation2 (Nav2) 构建的多 AGV 智能仓储物流调度与协作仿真系统。本项目融合了多机路径规划、走廊段路权预约制冲突解决、基于 YOLOv8 的视觉目标识别以及直观的 Web 监控面板。

## 🚀 系统特性

1. **双重导航驱动切换**：
   - **Nav2 工业导航**：支持 AMCL 定位、Costmap 避障规划以及行为树 (BT) 路径搜索。
   - **真值跟随回退导航**：轻量级仿真真值驱动方案，通过直线 P 控制算法进行点对点巡航，降低 CPU 开销。
2. **走廊路权预约机制 (Right-of-Way)**：
   - 解决多车在狭窄单行道（走廊段）的死锁与冲突问题，通过预约锁段机制自动让行，保障多车平稳运转。
3. **AI 视觉目标识别**：
   - 使用 YOLOv8 对摄像头画面进行实时物体检测，感知场景中的特定目标物。
4. **可视化 Web 面板**：
   - 基于 HTML/JS + Rosbridge，实时渲染仓库地图、AGV 位置、电量、当前任务、路权占用，并支持一键下发运输任务与手动控制。

## 📂 工作空间结构

```text
.
├── src/
│   ├── amr_description/       # AGV 模型 URDF、Gazebo 世界描述、Nav2 导航桥接与调度中心(fleet_manager)
│   ├── amr_vision/            # 基于 YOLOv8 的视觉识别模块与摄像头控制
│   └── amr_web/               # Rosbridge Web 监控界面与静态网页
├── doc/
│   ├── operations-guide.md                 # 完整系统操作指南
│   ├── navigation-modes-and-fallback.md    # 导航模式与真值回退架构设计
│   ├── right-of-way-design.md              # 走廊段路权预约逻辑规范
│   └── mapping-guide.md                    # 2D 激光雷达建图指导
└── README.md                  # 本文档
```

## 🛠️ 快速启动

详细安装与配置步骤请参阅 [系统操作指南](file:///home/li/GazeboLib/doc/operations-guide.md)。

### 1. 依赖安装
系统需要 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11。
```zsh
sudo apt update
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs ros-humble-rosbridge-server ros-humble-xacro python3-transforms3d
python3 -m pip install --user -U "transforms3d>=0.4.2"
```

### 2. 编译项目
请在项目根目录下使用顺序执行器进行编译以避免并行冲突：
```zsh
cd ~/GazeboLib
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --executor sequential
```

### 3. 一键启动
在 WSL/Ubuntu 终端运行（默认启动 2 台 AGV + Nav2 导航 + Web 面板）：
```zsh
source install/setup.zsh
ros2 launch amr_description warehouse_fleet_nav2.launch.py
```
- 启动真值跟随模式（轻量化调试）：
  ```zsh
  ros2 launch amr_description warehouse_fleet.launch.py
  ```

### 4. 访问 Web 监控面板
启动后等待 30-60 秒直至 Nav2 激活，在浏览器中打开：
```text
http://localhost:8080/
```

## 📖 相关文档
更多深入细节和设计方案，请阅读：
- [系统操作与排障指南 (operations-guide.md)](file:///home/li/GazeboLib/doc/operations-guide.md)
- [双重导航驱动与就绪判断设计 (navigation-modes-and-fallback.md)](file:///home/li/GazeboLib/doc/navigation-modes-and-fallback.md)
- [走廊路权锁段预约设计 (right-of-way-design.md)](file:///home/li/GazeboLib/doc/right-of-way-design.md)
- [仓库 SLAM 建图指引 (mapping-guide.md)](file:///home/li/GazeboLib/doc/mapping-guide.md)
