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

详细安装与配置步骤请参阅 [系统操作指南](./doc/operations-guide.md)。

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
git clone --branch WSLHumble https://github.com/heisd/AGV-Car-Solve-.git
cd AGV-Car-Solve-
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
- [系统操作与排障指南 (operations-guide.md)](./doc/operations-guide.md)
- [双重导航驱动与就绪判断设计 (navigation-modes-and-fallback.md)](./doc/navigation-modes-and-fallback.md)
- [走廊路权锁段预约设计 (right-of-way-design.md)](./doc/right-of-way-design.md)
- [仓库 SLAM 建图指引 (mapping-guide.md)](./doc/mapping-guide.md)

## 两种导航模式的边界

| 项目 | Nav2 模式 | 仿真真值跟随模式 |
| --- | --- | --- |
| 入口 | warehouse_fleet_nav2.launch.py | warehouse_fleet.launch.py |
| 位姿与导航链路 | 使用定位、规划与 Nav2 接口 | 使用仿真真值和轻量跟随逻辑 |
| 适合用途 | 完整导航集成与多车调试 | 快速演示、资源受限的流程检查 |
| 避障理解 | 需检查地图、代价地图与规划行为 | 前向减速不能视为完整绕障能力 |
| 面板导航就绪提示 | 反映 Nav2 就绪状态 | 可能持续显示未就绪，不能直接据此判定整个系统故障 |

两种模式的设计和状态解释见 [导航模式说明](./doc/navigation-modes-and-fallback.md)。真值驱动属于仿真便利机制，不能直接作为实机导航能力的验证结果。

## 启动参数与调试范围

操作指南记录的常用参数如下，实际可用参数可通过 launch 的帮助信息核对：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| num_agvs | 2 | 车队数量 |
| gui | true | 是否显示 Gazebo 界面 |
| use_web | true | 是否启动 Web 面板 |
| world | 包内仓库场景 | 指定世界文件路径 |

```zsh
ros2 launch amr_description warehouse_fleet_nav2.launch.py --show-args
ros2 launch amr_description warehouse_fleet_nav2.launch.py num_agvs:=1 gui:=false use_web:=false
```

首次定位故障时可缩小到单车；完整演示再恢复默认两车。增加车辆数量会增加仿真与导航计算量，项目没有对所有硬件配置给出性能保证。

本页使用 zsh 示例；如果实际终端是 Bash，请使用对应的 `setup.bash`。不要只修改代码块语言标记而继续加载不匹配的环境脚本。

## 下发第一条运输任务

在 Web 面板选择取货点与卸货点，提交任务后观察车队状态、任务变化和地图路径。文档中的示例区域包括：

| 名称 | 作用 |
| --- | --- |
| pickup_A / pickup_B | 取货点 |
| dropoff_A / dropoff_B | 卸货点 |
| charger_1 | 充电点 |

操作指南描述了从 IDLE 车辆进行手动导航的行为；执行任务中的车不应被当作空闲车继续下发手动目标。区域坐标和服务接口应以当前配置及指南为准。

## 分层排障

```zsh
ros2 node list
ros2 topic list -t
ros2 topic echo /fleet/state --field data --once
ros2 topic echo /agv1/plan --once
```

| 现象 | 检查顺序 |
| --- | --- |
| Gazebo 中没有车 | 场景加载、spawn 与启动日志 |
| 面板打不开 | 网页服务是否启动、8080 端口是否可访问 |
| 网页打开但没有车队数据 | rosbridge 与 ROS 节点、话题及浏览器连接 |
| Nav2 未就绪 | 仿真时钟、TF、定位、生命周期与资源占用 |
| 车辆停在走廊 | 先区分正常路权等待、让行与规划失败 |
| 目标不可达 | 地图、障碍物、目标坐标与路径结果 |

停止时优先在启动终端按 `Ctrl+C`，再确认相关进程退出。重启前检查是否有上一轮残留节点，避免同名节点、重复 TF 或端口占用影响判断。

## 进一步阅读

- [操作、配置与常见故障](./doc/operations-guide.md)
- [导航模式与回退机制](./doc/navigation-modes-and-fallback.md)
- [走廊路权设计](./doc/right-of-way-design.md)
- [SLAM 建图指引](./doc/mapping-guide.md)

本项目用于仓库仿真与算法集成学习。这里的功能与启动方法依据仓库实现和现有指南整理，未宣称经过实机迁移或全部场景的无死锁验证。
