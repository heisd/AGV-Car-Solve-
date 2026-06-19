# 系统操作指南

> 多 AGV 仓库仿真系统 — 从零启动到运行的完整操作手册  
> 更新日期：2026-06-19

---

## 目录

1. [环境要求](#1-环境要求)
2. [首次安装](#2-首次安装)
3. [编译项目](#3-编译项目)
4. [启动系统（推荐方式）](#4-启动系统推荐方式)
5. [打开 Web 操作面板](#5-打开-web-操作面板)
6. [可选启动方式](#6-可选启动方式)
7. [常用操作](#7-常用操作)
8. [停止系统](#8-停止系统)
9. [排障指南](#9-排障指南)
10. [文件地图](#10-文件地图)

---

## 1. 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04（WSL2 或原生均可） |
| ROS 版本 | ROS 2 **Humble** |
| 仿真器 | Gazebo **Classic** 11（`/usr/bin/gazebo`，非 Ignition） |
| Shell | **zsh**（本项目默认使用 zsh） |
| 内存 | ≥ 8GB（2 车稳定），≥ 16GB（3 车推荐） |
| 显示 | WSLg 或 X Server（Gazebo GUI 需要图形环境） |

### 依赖包

```zsh
# ROS 2 核心
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup

# Gazebo Classic
sudo apt install ros-humble-gazebo-ros-pkgs

# rosbridge（Web 面板需要）
sudo apt install ros-humble-rosbridge-server

# 其他依赖
sudo apt install ros-humble-xacro python3-transforms3d

# Python 依赖（transforms3d 需 ≥ 0.4.2 兼容 numpy）
python3 -m pip install --user -U "transforms3d>=0.4.2"
```

---

## 2. 首次安装

```zsh
# 进入工程目录
cd ~/GazeboLib

# 安装 ROS 依赖
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3. 编译项目

> ⚠️ **每次修改代码后都需要重新编译。**

```zsh
cd ~/GazeboLib

# 1. Source ROS 环境
source /opt/ros/humble/setup.zsh

# 2. 编译（使用顺序编译器，避免并行冲突）
colcon build --symlink-install --executor sequential

# 3. Source 工作区（⚠️ zsh 必须用 setup.zsh，不能用 setup.bash！）
source install/setup.zsh
```

### 快速确认编译成功

```zsh
# 应该列出 3 个包
colcon list
# 输出：
#   amr_description
#   amr_vision
#   amr_web
```

---

## 4. 启动系统（推荐方式）

### 一键启动（Nav2 驱动，2 车 + Web 面板）

```zsh
cd ~/GazeboLib
source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 launch amr_description warehouse_fleet_nav2.launch.py
```

这条命令会自动启动：
- ✅ Gazebo 仿真器 + 仓库世界（20 个货架 + 取货/卸货/充电站）
- ✅ 2 台 AGV（agv1、agv2）+ 全套传感器和辅助节点
- ✅ 每车一套 Nav2 导航栈（路径规划 + 动态避障）
- ✅ 调度中心 fleet_manager（走廊段预约制路权 + 任务分配）
- ✅ Web 操作面板（rosbridge + 静态网页服务器）

### 启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `world` | `<pkg>/worlds/warehouse.world` | Gazebo 仿真场景世界的绝对路径 |
| `num_agvs` | `2` | AGV 数量（1~3）。默认 2 台最稳定 |
| `use_web` | `true` | 是否启动 Web 面板 |
| `gui` | `true` | 是否显示 Gazebo 3D 界面 |

### 示例

```zsh
# 默认：2 车 + Web + GUI
ros2 launch amr_description warehouse_fleet_nav2.launch.py

# 指定自定义的 Gazebo World 仿真场景世界
ros2 launch amr_description warehouse_fleet_nav2.launch.py world:=/path/to/your/custom.world

# 3 车（需要较好的机器配置）
ros2 launch amr_description warehouse_fleet_nav2.launch.py num_agvs:=3

# 无头模式（无 Gazebo GUI，适合服务器/压测）
ros2 launch amr_description warehouse_fleet_nav2.launch.py gui:=false

# 不启动 Web 面板
ros2 launch amr_description warehouse_fleet_nav2.launch.py use_web:=false

# 1 台车 + 无头 + 无 Web（最轻量级调试）
ros2 launch amr_description warehouse_fleet_nav2.launch.py num_agvs:=1 gui:=false use_web:=false
```

---

## 5. 打开 Web 操作面板

系统启动后（等待约 30~60 秒让 Nav2 完全激活），在浏览器中打开：

```
http://localhost:8080/
```

### 面板功能

| 区域 | 功能 |
|------|------|
| **仓库地图** | 实时显示 AGV 位置、朝向、电量环、Nav2 规划路径、走廊段占用 |
| **车队状态表** | 每台车的状态、电量、任务、导航就绪状态 |
| **任务下发** | 选择取货点和卸货点，提交运输任务 |
| **系统异常** | 实时显示碰撞、低电量、导航卡死、路权等待等异常 |
| **异常日志** | 订阅 /rosout 显示系统 WARN/ERROR 级别日志 |

### 下发任务

在 Web 面板的任务下发表单中：
1. 选择 **取货点**（pickup_A 或 pickup_B）
2. 选择 **卸货点**（dropoff_A 或 dropoff_B）
3. 点击 **提交**

调度中心会自动将任务分配给空闲的 AGV。

### 手动导航

1. 在车队状态表中**点击选中**一台 IDLE 状态的车
2. 在地图上**点击目标位置**
3. 系统会直接发送 Nav2 导航目标

> ⚠️ 只有 IDLE 状态的车才允许手动导航，执行任务中的车会被拦截。

---

## 6. 可选启动方式

### 6.1 真值跟随器驱动（不使用 Nav2，更轻量）

```zsh
ros2 launch amr_description warehouse_fleet.launch.py
```

- 不使用 Nav2 路径规划，AGV 直线奔向目标
- 仅靠激光前向减速避障，无真实绕障
- 适合快速演示或机器配置不足时
- **说明**：在此模式下，Web 面板会持续显示“导航未就绪”（红标），这是正常现象。关于这两种导航模式及状态判定逻辑的详细剖析，请阅读 [AGV 导航模式与双重回退驱动说明文档](file:///home/li/GazeboLib/doc/navigation-modes-and-fallback.md)。

### 6.2 单车调试

```zsh
ros2 launch amr_description warehouse_bringup.launch.py
```

- 只启动 Gazebo + 1 台 AGV，不启动调度中心
- 适合调试单车 URDF、传感器、Nav2 配置

```zsh
# 带 RViz 可视化
ros2 launch amr_description warehouse_bringup.launch.py use_rviz:=true

# 指定初始位置（在充电站）
ros2 launch amr_description warehouse_bringup.launch.py x:=-5.5 y:=-5.5
```

### 6.3 仅启动 Web 面板

```zsh
ros2 launch amr_web web_panel.launch.py
```

- 单独启动 rosbridge + 网页服务器
- 适合后端已在运行时，只需要重启前端

---

## 7. 常用操作

### 7.1 查看系统状态

```zsh
# 查看车队状态 JSON（调度中心发布，~3Hz）
ros2 topic echo /fleet/state --field data --once | python3 -m json.tool

# 查看所有活跃话题
ros2 topic list | grep -E "agv|fleet"

# 查看某台车的导航状态
ros2 topic echo /agv1/goal_pose --once

# 查看 Nav2 规划路径
ros2 topic echo /agv1/plan --once
```

### 7.2 手动添加任务

```zsh
# 通过命令行下发运输任务
ros2 topic pub --once /fleet/add_task std_msgs/msg/String \
  "data: '{\"pickup\": \"pickup_A\", \"dropoff\": \"dropoff_A\"}'"

# 带自定义任务 ID 和装卸时间
ros2 topic pub --once /fleet/add_task std_msgs/msg/String \
  "data: '{\"id\": \"MY_TASK_1\", \"pickup\": \"pickup_B\", \"dropoff\": \"dropoff_B\", \"load_time\": 5.0, \"unload_time\": 5.0}'"
```

### 7.3 可用区域

| 区域名 | 位置 | 用途 |
|--------|------|------|
| `pickup_A` | (5.5, 5.5) | 东北角取货点 |
| `pickup_B` | (-5.5, 5.5) | 西北角取货点 |
| `dropoff_A` | (5.5, -5.5) | 东南角卸货点 |
| `dropoff_B` | (-5.5, -2.5) | 西侧走廊卸货点 |
| `charger_1` | (-5.5, -5.5) | 西南角充电站 |

### 7.4 查看走廊段路权状态

```zsh
# 从 /fleet/state 提取走廊段占用信息
ros2 topic echo /fleet/state --field data --once 2>/dev/null | \
  python3 -c "
import sys, json
d = json.loads(sys.stdin.read().strip().strip(\"'\"))
print('走廊段占用:', json.dumps(d.get('segment_owner',{}), indent=2, ensure_ascii=False))
print('等待队列:', json.dumps(d.get('segment_queue',{}), indent=2, ensure_ascii=False))
print('让行中:', d.get('yielding',[]))
"
```

---

## 8. 停止系统

### ✅ 正确方式：Ctrl+C

在启动 `ros2 launch` 的终端中按 **Ctrl+C**。

ROS 2 launch 会收到 SIGINT 信号，**优雅关闭所有子节点**（Nav2 容器、Gazebo、fleet_manager、Web 服务器等）。

### ❌ 错误方式：切勿使用 `kill -9`

```zsh
# ❌ 千万不要这样做！！
kill -9 <launch_pid>
```

> **原因**：`kill -9`（SIGKILL）不会传播给子进程，所有 Nav2 容器、odom_sim_filter、
> battery_sim、tf_relay 等节点会变成**孤儿僵尸进程**，持续占用 CPU 和发布 /tf，
> 导致下次启动时 TF 混乱、AMCL 定位失败、load 虚高。
> 详见构建日志第 17 条问题。

### 如果已经误用了 `kill -9`

```zsh
# 清理所有残留 ROS 节点
pkill -f "fleet_manager|odom_sim_filter|battery_sim|charger_dock_monitor|tf_relay|nav2_goal_bridge"
pkill -f "component_conta"    # Nav2 容器（进程名被截断为 15 字符）
pkill -f "gzserver|gzclient"  # Gazebo

# 确认清理干净
ps aux | grep -E "ros|gazebo|fleet" | grep -v grep
```

---

## 9. 排障指南

### 9.1 Gazebo 启动慢 / 车没有生成

**现象**：Gazebo 加载世界超过 30 秒，AGV 未出现在仿真中。

**原因**：20 个货架模型加载耗时，`spawn_entity` 等待服务就绪。

**解决**：耐心等待。launch 已将 spawn 超时设为 120 秒，并错峰启动（8/11/14s）。

### 9.2 Nav2 导航未就绪（Web 面板标红）

**现象**：Web 面板车队表显示某台车「导航未就绪」，AGV 不动。

**可能原因**：
- 3 台车时第 3 套 Nav2 组合节点加载超时（8GB 内存极限）
- spawn 尚未完成，Nav2 还在等待

**解决**：
- 等待 1~2 分钟看是否自行恢复
- 如果持续不就绪：改用 `num_agvs:=2`（2 车完全稳定）
- 或重启系统

### 9.3 AGV 不动 / 导航卡死

**现象**：AGV 停在原地不动，Web 面板显示「导航卡死」。

**可能原因**：
- 路权系统检测到走廊段冲突，AGV 在等待点让行（正常行为）
- Nav2 路径规划失败（目标在障碍物内）
- 前方有障碍物阻挡

**排查步骤**：
```zsh
# 查看该车当前状态
ros2 topic echo /fleet/state --field data --once 2>/dev/null | \
  python3 -c "import sys,json; d=json.loads(sys.stdin.read().strip().strip(\"'\")); [print(a) for a in d['agvs']]"

# 查看系统异常
ros2 topic echo /fleet/state --field data --once 2>/dev/null | \
  python3 -c "import sys,json; d=json.loads(sys.stdin.read().strip().strip(\"'\")); [print(a) for a in d['anomalies']]"
```

### 9.4 zsh 下 `source install/setup.bash` 报错

**现象**：`source install/setup.bash` 报找不到 `/home/li/GazeboLib/setup.sh`。

**原因**：colcon 的 `setup.bash` 依赖 `BASH_SOURCE`，zsh 下解析错误。

**解决**：zsh **必须用** `source install/setup.zsh`。

### 9.5 编译报错 `Failed/Aborted`

**现象**：`colcon build` 报某个包 Failed 或 Aborted。

**解决**：使用顺序编译器避免并行冲突：
```zsh
colcon build --symlink-install --executor sequential
```

---

## 10. 文件地图

```
GazeboLib/                          ← colcon 工作区根目录
├── src/
│   ├── amr_description/            ← 主包：AGV 模型 + 节点 + 导航 + 调度
│   │   ├── amr_description/
│   │   │   ├── fleet_manager.py    ← ★ 调度中心（路权系统、任务分配、碰撞检测）
│   │   │   ├── nav2_goal_bridge.py ← Nav2 导航桥接（goal_pose → NavigateToPose）
│   │   │   ├── battery_sim.py      ← 电池仿真
│   │   │   ├── charger_dock_monitor.py ← 充电对接检测
│   │   │   ├── odom_sim_filter.py  ← 里程计滤波 + TF 发布
│   │   │   ├── tf_relay.py         ← TF 命名空间转发
│   │   │   ├── ground_truth_waypoint_follower.py ← 真值跟随器（非 Nav2 模式）
│   │   │   └── obstacle_detection_node.py        ← 激光障碍检测
│   │   ├── launch/
│   │   │   ├── warehouse_fleet_nav2.launch.py  ← ★ 主入口（Nav2 驱动）
│   │   │   ├── warehouse_fleet.launch.py       ← 备选入口（真值跟随器）
│   │   │   └── warehouse_bringup.launch.py     ← 单车调试入口
│   │   ├── yaml/
│   │   │   ├── warehouse_tasks.yaml  ← 区域定义 + 初始任务
│   │   │   └── nav2_params_amr.yaml  ← Nav2 参数模板
│   │   ├── maps/
│   │   │   ├── warehouse_map.yaml    ← Nav2 地图元数据
│   │   │   └── warehouse_map.pgm     ← Nav2 地图图像
│   │   ├── urdf/
│   │   │   └── amr.urdf.xacro        ← AGV 机器人描述
│   │   ├── worlds/
│   │   │   └── warehouse.world        ← 仓库仿真世界
│   │   └── behavior_trees/
│   │       └── navigate_w_recovery.xml ← Nav2 行为树
│   ├── amr_vision/                 ← YOLO 视觉检测包
│   └── amr_web/                    ← Web 操作面板包
│       ├── web/
│       │   ├── index.html           ← 面板页面
│       │   ├── app.js               ← 面板逻辑
│       │   ├── style.css            ← 面板样式
│       │   └── vendor/roslib.min.js ← roslibjs（离线可用）
│       └── launch/
│           └── web_panel.launch.py  ← Web 面板独立启动
├── doc/
│   ├── operations-guide.md          ← ★ 本文档（操作指南）
│   ├── right-of-way-design.md       ← 路权系统设计文档
│   ├── navigation-modes-and-fallback.md ← 导航状态与回退驱动说明文档
│   ├── mapping-guide.md             ← 仓库建图与地图优化说明文档
│   ├── build-log.md                 ← 构建记录与排障日志
│   └── amr-ros-upstream/            ← 上游项目文档（溯源）
├── worlds/
│   └── warehouse.world              ← 原始仓库世界文件
├── build/                           ← colcon 编译输出
├── install/                         ← colcon 安装输出
├── log/                             ← colcon 日志
└── yolov8n.pt                       ← YOLO 模型权重
```

---

## 快速启动速查卡

```zsh
# === 完整启动流程（复制粘贴即可） ===

cd ~/GazeboLib
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --executor sequential
source install/setup.zsh
ros2 launch amr_description warehouse_fleet_nav2.launch.py

# 等待 30~60 秒后，浏览器打开：
# http://localhost:8080/

# 停止：在终端按 Ctrl+C
```
