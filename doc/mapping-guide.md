# AGV 仓库建图（SLAM）与高精度地图说明文档

> 版本：v1.0 — 建图操作与地图优化指南  
> 更新日期：2026-06-19  
> 适用于：GazeboLib 仓库仿真项目（ROS2 Humble + SLAM Toolbox）

---

## 目录
1. [建图（SLAM）操作指南](#1-建图slam操作指南)
2. [为什么扫描的地图 (my_map) 会有误差？](#2-为什么扫描的地图-my_map-会有误差)
3. [高精度真值地图 (warehouse_map) 推荐](#3-高精度真值地图-warehouse_map-推荐)
4. [地图切换与编译部署](#4-地图切换与编译部署)

---

## 1. 建图（SLAM）操作指南

本项目基于 `slam_toolbox` 提供了完整的激光雷达 SLAM 建图工具链。在 `zsh` 环境中，可以通过以下三个步骤扫描并保存新地图。

### 1.1 步骤一：启动建图仿真环境
在一个新终端中运行建图 Launch 文件。该脚本将启动 Gazebo 仿真环境（加载 `map.world` 场景）、一辆配置了单线激光雷达的 `agv1` 小车、SLAM 节点以及 RViz2 可视化界面：
```zsh
cd ~/GazeboLib
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch amr_description mapping.launch.py
```

### 1.2 步骤二：遥控小车扫描地图
在第二个终端中启动键盘遥控节点，通过键盘按键（如 `i`/`,`/`j`/`l`）控制小车在仓库内慢速行驶，使其雷达光束扫描到所有的墙壁、货架和障碍物：
```zsh
source /opt/ros/humble/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/agv1/cmd_vel
```
> **💡 建图技巧**：  
> - **车速宜慢不宜快**：建议设置线速度 `< 0.3 m/s`，角速度 `< 0.5 rad/s`。  
> - **平滑转向**：避免急停急转。急剧的运动会导致小车轮胎在 Gazebo 物理引擎中发生滑移（Slip），引起里程计定位累计误差，导致地图重影或断裂。  
> - **小幅修正**：当检测到局部有轻微错位时，可以将车倒回开阔区域，让 SLAM 进行回环检测（Loop Closure）自动修正地图。

### 1.3 步骤三：保存地图到源码目录
观察 RViz2 界面，确认所有区域均已扫描闭合且没有重影后，在第三个终端中执行地图保存工具，将生成的点阵图与配置文件存入源码目录中：
```zsh
source /opt/ros/humble/setup.zsh
ros2 run nav2_map_server map_saver_cli -f ~/GazeboLib/src/amr_description/maps/my_map
```
该命令将在 `src/amr_description/maps` 下生成两个文件：
- `my_map.pgm`：栅格地图二进制图像文件。
- `my_map.yaml`：描述地图元数据（如分辨率、原点、阈值等）的配置文件。

---

## 2. 为什么扫描的地图 (my_map) 会有误差？

在手动控制小车建图时，所得到的 `my_map` 可能会出现墙壁不够直、边缘有毛刺、甚至通道宽度发生微小漂移等情况。这是 SLAM 算法在物理仿真环境中的正常生理特性，主要成因包括：

1. **传感器噪声（Sensor Noise）**：激光雷达本身的测距存在统计学随机噪声。
2. **里程计滑移（Odometry Drift）**：仿真底盘在与地面摩擦、加减速时轮子存在微小打滑，累积计算位姿时会产生漂移。
3. **环境特征单一（Symmetric Environment）**：走廊过于狭窄且左右对称，导致雷达扫描得到的水平激光点在某些轴向上的约束度低，算法容易误判小车前进距离。

在 SLAM 精度不足的情况下，Nav2 导航栈在窄走廊内通行时可能会因为地图不精细而频发避障减速、定位粒子发散或者路径规划失败。

---

## 3. 高精度真值地图 (warehouse_map) 推荐

针对手动建图精度难以达到 100% 直线网格的问题，本项目专门设计了一套**高精度点阵真值地图**：
* **地图路径**：[warehouse_map.yaml](file:///home/li/GazeboLib/src/amr_description/maps/warehouse_map.yaml) / [warehouse_map.pgm](file:///home/li/GazeboLib/src/amr_description/maps/warehouse_map.pgm)
* **设计原理**：该地图没有采用物理扫描，而是通过读取 `worlds/warehouse.world` 中墙壁与货架的绝对几何尺寸，由纯数学矩阵算法确定性渲染生成的 1:1 无畸变栅格地图。
* **优势**：所有通道宽度绝对恒定，货架和墙壁呈完全垂直/水平的像素直线，不存在任何环境噪声。这能够让 Nav2 的定位算法（AMCL）和全局/局部路径规划器在多车狭窄通道穿行时表现出最优的通行稳定性和精度，彻底杜绝手动建图错位导致撞墙的隐患。

---

## 4. 地图切换与编译部署

由于 ROS 2 在运行时读取的是编译打包后的资源，无论是新扫描的 `my_map` 还是真值 `warehouse_map`，每次替换或保存后**必须重新进行 colcon 构建**以刷新 install 目录。

### 4.1 编译部署命令
```zsh
cd ~/GazeboLib
colcon build --symlink-install --packages-select amr_description
source install/setup.zsh
```

### 4.2 多车调度系统中的配置修改
在多车导航启动文件 [warehouse_fleet_nav2.launch.py](file:///home/li/GazeboLib/src/amr_description/launch/warehouse_fleet_nav2.launch.py#L45) 中，默认已配置使用高精度的 `warehouse_map.yaml`：
```python
map_file = os.path.join(pkg_amr, 'maps', 'warehouse_map.yaml')
```
如果您想要测试自己扫描的新地图，只需要将上述行修改为 `my_map.yaml`：
```python
map_file = os.path.join(pkg_amr, 'maps', 'my_map.yaml')
```
修改完成后，重复上述的编译命令即可。
