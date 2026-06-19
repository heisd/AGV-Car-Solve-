# 仓库 Gazebo 世界 — 构建记录

> 记录搭建过程中的关键决策与遇到的问题。持续更新。

## 1. 环境信息

| 项目 | 值 |
|------|-----|
| 仿真器 | Gazebo **Classic** 11.10.2（`/usr/bin/gazebo`，非 Ignition/gz-sim） |
| ROS | ROS 2 **Humble**（`/opt/ros/humble`） |
| 模型库 | `~/.gazebo/models`（Gazebo Classic 自动搜索此目录，`model://` 可直接解析） |
| 工程目录 | `/home/li/GazeboLib`（初始为空目录） |
| 平台 | WSL2 (Ubuntu) |

## 2. 场景设计

世界文件：`worlds/warehouse.world`（SDF 1.6，world 名 `warehouse`）。

### 布局（俯视，单位 m，原点在仓库中心）
- **仓库外墙** `warehouse_building`：内部 14×14 m，墙厚 0.2 m，高 3 m（灰色）。
- **20 个货架** `shelf_00`..`shelf_19`：4 排 × 5 列。
  - 列方向 X：`{-2, -1, 0, 1, 2}`（中心间距 1.0 m，货架宽 0.9 m → 间隙 0.1 m）。
  - 排方向 Y：`{4.5, 1.5, -1.5, -4.5}`（排间通道约 2.6 m，便于后续 AGV 通行）。
- **取货点** `pickup_station`（东北角 `5.5, 5.5`）：绿色地标 2.5×2.5 m + `euro_pallet`（欧标托盘）+ `grey_tote`（料箱）。
- **充电站** `charging_station`（西南角 `-5.5, -5.5`）：橙色地标 2.0×2.0 m + 黄色充电柜 + 黑色充电触点柱。

### 货架模型选型
`~/.gazebo/models` 中**没有**专用的 shelf/rack/货架模型，复用了 `bookshelf`：
- 尺寸：宽 0.9 (X) × 深 0.4 (Y) × 高 1.2 (Z) m，**原点在底部地面**。
- 开口朝 **-Y** 方向（背板在 +Y 侧）；当前所有货架朝向一致（yaw=0）。

## 3. 过程中的问题与决策

1. **无专用货架模型** → 选用 `bookshelf` 代替，已记录其尺寸与开口朝向，便于后续摆放货物 / 调整朝向。
2. **无专用取货点/充电站模型** → 用内联自定义 model（带颜色地标 + 几何体）+ 复用 `euro_pallet`、`grey_tote` 拼装，自包含、无额外依赖。
3. **`gz model --list` 在 Classic 中报 "Invalid arguments"** → Gazebo Classic 的 `gz model` 不支持 `--list`；改用 `gz model -m <name> -p` 查询单个模型位姿来验证。
4. **`pickup_pallet` 的 Z 轻微下沉（0.068 m）** → `euro_pallet` 是带物理的非静态模型，受重力轻微沉降，属正常现象；如需固定可后续设为 `static` 或放到静态底座上。
5. **`gz model --list` 之外的存活验证** → 通过 `gz topic -l` 看到 `/gazebo/warehouse/...` 话题，确认世界已正常运行。

## 4. 验证结果

- `gz sdf -k worlds/warehouse.world` → **Check complete**（语法通过）。
- 依赖模型 `sun / ground_plane / bookshelf / euro_pallet / grey_tote` → 均存在于 `~/.gazebo/models`。
- `gzserver --verbose worlds/warehouse.world` headless 加载 → 无 error/warning。
- `gz model -m <name> -p` 抽查 → `shelf_00 (-2,4.5)`、`shelf_19 (2,-4.5)`、`pickup_station (5.5,5.5)`、`charging_station (-5.5,-5.5)`、`warehouse_building (0,0)` 位姿全部正确。
- 结论：**23 个模型全部正常加载，世界可用。**

## 5. 启动方式

```bash
cd /home/li/GazeboLib
gazebo worlds/warehouse.world            # 带 GUI（需 WSLg/X server）
gzserver worlds/warehouse.world          # 仅服务端（headless）
```

## 6. AGV 模型集成（GitHub 克隆）

从 GitHub 选用 **amr-ros**（多 AGV 车队框架，与本仓库的充电站/取货点主题契合）。

- 来源：`https://github.com/shrikrishnarb/amr-ros.git`
- 克隆 commit：`ba0d44c9ff70fa0e8cb9343846af5d5f62e05f4f`（2026-05-27 "Updated readme"）
- 许可证：MIT（已保留至 `doc/amr-ros-upstream/LICENSE`）
- 机器人：差速驱动 AMR/AGV，含 2D/3D LiDAR、相机、IMU；附 Nav2 导航 + 多车队管理 + 电量监控 + 自动充电 + 任务调度。

### 按 ROS2 规范重整目录
上游把包放在 `colcon_ws/src/`；按需求迁移到工程根的标准 colcon 布局 `src/`：

```text
GazeboLib/                 <- colcon 工作区根
├── src/
│   ├── amr_description/   <- ament_python：URDF/世界/launch + 节点(fleet_manager, battery_sim, charger_dock_monitor, nav2_goal_bridge, ...)
│   └── amr_vision/        <- ament_python：YOLO 相机检测
├── worlds/warehouse.world <- 自建仓库世界
├── doc/
│   ├── build-log.md
│   └── amr-ros-upstream/  <- 上游 README/架构文档/媒体/docker + LICENSE（溯源用）
└── yolov8n.pt             <- amr_vision 的 YOLO 权重默认相对路径
```

### 过程问题与决策（AGV 部分）
6. **上游 `colcon_ws/src` 嵌套布局** → 与本工程的 ROS2 工作区习惯不符；将两个包 `mv` 到工程根 `src/`，删除空壳 `amr-ros/`（含其 `.git` 与混入的 `.omc/`）。
7. **`yolov8n.pt` 原在 `colcon_ws/` 根** → `amr_vision` 以相对默认路径 `yolov8n.pt` 引用；移到工作区根 `GazeboLib/yolov8n.pt` 以保持默认可用（缺失时 ultralytics 也会自动下载）。
8. **校验** → `source /opt/ros/humble/setup.bash && colcon list` 正确列出 `amr_description`、`amr_vision`（均 `ros.ament_python`），布局合规。
9. **`rosdep install` 报 `Cannot locate rosdep definition for [ament_python]`**（上游 bug）→ `amr_vision/package.xml` 误将 `ament_python` 写成 `<buildtool_depend>`，但它不是 rosdep 系统依赖键，构建类型应只在 `<export><build_type>ament_python</build_type></export>` 声明。删除该 `<buildtool_depend>` 行后重跑 → `#All required rosdeps installed successfully`，无报错。

## 7. 系统启动 Launch

新建 `src/amr_description/launch/warehouse_bringup.launch.py`——单车进仓库的总入口：
- 起 Gazebo Classic + **我们的** `warehouse.world`（已 `cp` 进包 `worlds/`，随 `colcon build` 安装到 share）。
- spawn 一台 AGV（差速 + 2D 激光），默认位姿中央通道 `(0,0,0.1)`。
- 起仿真辅助节点：`odom_sim_filter`、`battery_sim`、`charger_dock_monitor`（充电桩坐标已对到本世界充电站 `(-5.5,-5.5)`）。
- 可选 RViz（`use_rviz:=true`）。
- 参数：`namespace / world / x / y / z / use_rviz`（外加 gazebo_ros 继承的 `gui / server`）。

### 启动命令
```bash
cd ~/GazeboLib
colcon build --symlink-install --executor sequential
source /opt/ros/humble/setup.bash
source install/setup.zsh        # zsh 用 .zsh；bash 用 install/setup.bash
ros2 launch amr_description warehouse_bringup.launch.py
# 例：起 RViz 并让 AGV 停在充电桩
ros2 launch amr_description warehouse_bringup.launch.py use_rviz:=true x:=-5.5 y:=-5.5
```

### 过程问题与决策（Launch/构建部分）
10. **并行 `colcon build` 失败**：两个 ament_python 包并行 `setup.py develop --symlink-install` 争用共享 install 目录，首次报 `Failed/Aborted`。→ 用 `--executor sequential`（或分包构建）即稳定通过。
11. **`source install/setup.bash` 在 zsh 下失效**（报找 `/home/li/GazeboLib/setup.sh`）：colcon 的 `setup.bash` 依赖 `BASH_SOURCE` 定位自身，zsh 下解析错误，`AMENT_PREFIX_PATH` 不含工作区，`ros2 launch` 找不到 `amr_description`。→ **zsh 用 `source install/setup.zsh`**，或在 bash 子shell 里 source。
12. **`world`/launch 安装校验**：`ros2 launch ... --show-args` 正常列出全部参数，默认 `world` 解析到 `install/.../share/amr_description/worlds/warehouse.world`，包可见、launch 可加载。

## 8. 多车 + 调度中心 + Web 面板

目标架构：**3 台 AGV + 调度中心 + Web 操作面板**，免建图（真值跟随器驱动）。

```
浏览器(Web面板) ──ws:9090(rosbridge)──► 调度中心 fleet_manager
                                          ├─ 发 /fleet/state (JSON 车队状态, 3Hz) ← 面板地图/表格
                                          ├─ 收 /fleet/add_task (JSON)            ← 面板下发任务
                                          └─ 发 /<ns>/goal_pose ─► ground_truth_waypoint_follower ─► /<ns>/cmd_vel
3×AGV(agv1..3): rsp + spawn + odom_sim_filter + battery_sim + charger_dock_monitor
                + obstacle_detection + ground_truth_waypoint_follower    @ warehouse.world
面板异常日志：订阅 /rosout (rcl_interfaces/msg/Log)，显示 WARN/ERROR/FATAL
```

### 改动 / 新增
- **`fleet_manager.py`（调度中心）**：新增对 Web 的接口（向后兼容）——
  - 发布 `/fleet/state`（`std_msgs/String` JSON，~3Hz）：每台车 `{ns,x,y,yaw,battery,state,task,carrying,on_charger}` + `queued_tasks` + `zones`。
  - 订阅 `/fleet/add_task`（`std_msgs/String` JSON `{pickup,dropoff}`）：校验区域后入队，非法则发 WARN。
  - 记录 yaw（从 ground_truth 四元数换算）。
- **`yaml/warehouse_tasks.yaml`**：匹配本仓库的 zones（pickup_A=5.5,5.5；charger_1=-5.5,-5.5 等）+ 2 个初始任务，路径沿四周走廊。
- **新包 `amr_web`**（ament_python）：`web/{index.html,app.js,style.css,vendor/roslib.min.js}`（roslibjs 本地内置，离线可用）+ `launch/web_panel.launch.py`（rosbridge_websocket + `python3 -m http.server`）。
  - 面板功能：仓库俯视实时地图（货架/取货点/充电站/区域/AGV 位姿+电量环）、车队状态表、任务下发表单、**异常日志区（/rosout，WARN+，可选含 INFO）**、连接状态、自动重连。
- **`amr_description/launch/warehouse_fleet.launch.py`**：一键总入口 = Gazebo + warehouse.world + 3 车全套节点 + 调度中心 + 含 Web 面板层（`use_web:=true`）。
- `amr_description/package.xml` 增加 `gazebo_ros / xacro / amr_web` 依赖。

### 关键话题对接（避免踩坑）
- `charger_dock_monitor` 必须把 `contact_topic` 设为 `/<ns>/on_charger`（fleet_manager 与 battery_sim 都按此名订阅）；`odom_topic=/<ns>/ground_truth`；`charger_centers_xy=[-5.5,-5.5]`。
- `battery_sim` 的 `charger_contact_topic` 也设为 `/<ns>/on_charger`，充电闭环才成立。
- 跟随器可执行名是 `ground_truth_waypoint_follower.py`（setup.py 入口点带 `.py`）。

### 验证（headless，不依赖 GUI）
- `colcon build --executor sequential` 三包通过（依赖序 amr_web→amr_description）。`py_compile` / `node --check` 通过。
- 两个 launch `--show-args` 正常；web 资源、`warehouse_tasks.yaml` 均已安装。
- 单独跑调度中心：`/fleet/state` 发布完整 JSON；`/fleet/add_task` 合法任务入队（`[dispatch] queued task W1`）；非法任务产生 `WARN`（→ 面板异常日志）。
- 静态网页服务器：`index.html / app.js / style.css / vendor/roslib.min.js` 全部 HTTP 200。

### 首次运行排障（GUI 实跑）
13. **Gazebo 里没有车 + follower 节点全缺**：两个独立问题。
    - **follower / odom_sim_filter 启动即崩**：根因是 `~/.local` 的新 numpy(1.24.4，已删 `np.float`) 与 apt 旧 `transforms3d` 冲突——`transforms3d/quaternions.py` 用了 `np.maximum_sctype(np.float)` → `AttributeError: module 'numpy' has no attribute 'float'`。哪个 numpy 先加载决定崩不崩（所以 odom_sim_filter 时好时坏）。
      **修复**：`python3 -m pip install --user -U "transforms3d>=0.4.2"`（新版不再用 `np.float`），4 个用到 `tf_transformations` 的文件零改动、彻底解决。
    - **3 台车未 spawn 进 Gazebo**：launch 里 spawn_entity 用 `-topic /<ns>/robot_description` 从 robot_state_publisher 的 latched 话题取描述；并发启动下 spawn_entity 的订阅拿不到已 latch 的 xml 而死等，最终不 spawn（手动单独 spawn 能成、`-file` 能成，印证是话题路径问题）。
      **修复**：`warehouse_fleet.launch.py` 改为在启动时用 `xacro` 展开成临时 URDF 文件，spawn 用 `-file`（确定性，不依赖话题）；同一份 URDF 字符串也直接喂给 robot_state_publisher。另保留 `TimerAction` 错峰 5/7/9s 等服务就绪 (~7.5s)。
    - 验证（headless 整链）：升级后 `tf_transformations` 在 numpy 1.24.4 下 import OK；`xacro` 生成 URDF 222 行无误；整套 launch 跑起来后 **agv1/agv2/agv3 的 `/<ns>/ground_truth` 发布者数均为 1**（三台都进 Gazebo），日志显示调度中心给 agv1 派发取货任务、follower 收到 goal 并驱动（撞货架时按预期触发激光急停）。

### 已知限制（决策记录）
- 用**真值跟随器**（用户选择，免建图）：直线驶向目标 + 激光前向减速，**无真实绕障路径规划**；任务区放在走廊以尽量不穿货架，仍可能在斜线路径上贴近货架而减速/停下。后续可升级 Nav2。

## 9. 下一步

- [ ] 带 GUI 跑 `ros2 launch amr_description warehouse_fleet.launch.py`，浏览器开 `http://localhost:8080/`，验证 3 车进仓库、面板地图/表格/日志刷新、下发任务能驱动 AGV。
- [ ] 调参：跟随器速度/到点阈值、任务区位置，减少穿货架。
- [x] **升级到 Nav2**：真实路径规划与动态避障（见第 10 节）。
- [ ] （可选）面板增强：手动指派某台车去某区、急停按钮、任务历史。

## 10. 升级 Nav2 + Composition 降耗 + 导航修复

目标：把驱动层从「真值跟随器」换成 **Nav2**（真实全局/局部路径规划 + 动态避障），并在**单机仿真**前提下把多车 CPU 开销压下来。

> 路线决策：用户明确**不走分布式方案**（每车一块嵌入式板各跑一套 Nav2，不符合单机仿真实情），
> 采用**计划 1 = Nav2 Composition（容器内通信）**：把每车一套 Nav2 节点合并进单个
> `component_container_isolated` 进程，启用进程内通信（IPC，零拷贝），每个节点各用一个
> 独立的 single-threaded executor（避免大 multi-threaded executor 的大锁竞争）。

```
每车一套命名空间化 Nav2 栈（全在 1 个 component_container_isolated 进程内，IPC）：
  map_server + amcl(定位) + planner_server(NavFn 全局) + controller_server(DWB 局部)
  + behavior_server(恢复) + bt_navigator + smoother + velocity_smoother + waypoint_follower
调度中心 fleet_manager ──/<ns>/goal_pose──► nav2_goal_bridge ──NavigateToPose──► Nav2
                                            └──播种 AMCL 初始位姿(=spawn 位姿)
地图：maps/warehouse_map.{pgm,yaml}（由仓库几何确定性生成，匹配 warehouse.world）
TF：map ─(AMCL)─► <ns>/odom ─(odom_sim_filter)─► <ns>/base_footprint ...
```

### 改动 / 新增
- **新 launch `warehouse_fleet_nav2.launch.py`**：Gazebo + 3 车（rsp + spawn(-file) + odom_sim_filter
  + battery_sim + charger_dock_monitor + **tf_relay** + 每车一套 Nav2 + `nav2_goal_bridge`）+ 调度中心 + 可选 Web。
  - 参数：`num_agvs`(默认 3)、`use_web`(默认 true)、**`gui`**(默认 true，headless 压测设 false)。
  - Nav2 经 `nav2_bringup/bringup_launch.py` 拉起，关键参数 **`use_composition:=True`**、`use_respawn:=False`。
  - 车间**错峰启动 12/24/36s**（避免 3 套 Nav2 同时 `load_node` 抢 DDS 服务）；spawn 错峰 4/6/8s。
- **`yaml/nav2_params_amr.yaml`**：`{NS}`/`{BT_FILE}` 占位符在 launch 时替换。已按多车降耗调参：
  controller 3Hz、DWB 仅 5×1×10=50 条轨迹、全局/局部 costmap 2/5Hz、AMCL `max_beams=60`。
- **`nav2_goal_bridge.py`**：把 `/<ns>/goal_pose` 转 `NavigateToPose` 动作；启动时按 spawn 位姿播种 AMCL
  初始位姿；缓冲「Nav2 激活前」收到的目标；忽略「同目标重发」以免抢占在跑的导航。
- **`maps/warehouse_map.{pgm,yaml}`**：320×320、分辨率 0.05、origin[-8,-8]，由墙体+货架几何生成。

### 过程问题与决策（Nav2 部分）
14. **3 车 Nav2 资源爆炸（非 composition）**：`use_composition:=False` 时每车 ~7 个独立进程、共 21+ 进程，
    16 核机器 load 冲到 **77**，啥都收敛不了。
    → **修复 = 计划 1**：`use_composition:=True` → **每车仅 1 个 `component_container_isolated` 进程**，
    进程内通信。实测 3 车稳定 load ≈ **7**（1 分钟峰值，5 分钟均值更低），内存 ~5.5Gi/7.8Gi。
    **降耗约 1 个数量级**。（启动瞬间的 6 容器是错峰过程中的定位/导航容器瞬态，稳定后每车 1 个。）
15. **取货点目标落在障碍中心 → 全局规划必失败**（核心导航 bug）：world 在 `(5.5,5.5)` 放了
    `euro_pallet` + `grey_tote` 实体；fleet 把导航目标直接设成取货点中心 `(5.5,5.5)`——正好在托盘正中。
    Nav2 全局 costmap 里该格是致命障碍 + 0.7m 膨胀层，`planner tolerance=0.5` 也找不到空闲格：
    `GridBased: failed to create plan to (5.50,5.50)` → 控制器 `No valid trajectories out of 54!`
    → 进度检查 `Failed to make progress` → fleet 每 10s 重发 → **抢占式重启**导航，靠每次重启间隙蹭
    一点点位移（几十轮、数分钟才挪到点）。非 Nav2 跟随器直接开过去、无 costmap，所以这坑从未暴露。
    → **修复**：给「中心落在障碍内」的 zone 增加可达**接近点 `gx,gy`**：
    `pickup_A` 取托盘正南走廊 `(5.5,3.8)`；`charger_1` 取充电柜正北 `(-5.5,-4.8)`（仍在
    `charger_dock_monitor` 的 0.8m 充电触发半径内）。`fleet_manager` 导航去 `gx,gy`、到达判定也按 `gx,gy`
    （缺省回退 `cx,cy`，空地 zone 无需配）。`gx,gy` 一并透传到 `/fleet/state` 供面板可视化。
16. **fleet_manager 定时盲重发 → 抢占风暴**：原 `_resend_goal_if_due` 每 10s 无条件重发当前目标
    （为兼容老的跟随器），Nav2 下每次重发 = 一个新 `NavigateToPose` = 抢占在跑的导航
    （`Received goal preemption request` 刷屏）。
    → **修复**：改为**基于停滞检测**——只有当「朝目标的距离在 12s 内无实质缩短（>0.3m）」才重发，
    作为「导航被中止/卡死」的重试；正常行驶不再重发（bridge 本就忽略同目标重发）。日志噪音消除、
    误导性文案（“may have missed initial publish”）去掉。
17. **僵尸进程累积 → load 假性爆炸 + 定位失败**（排障中踩的真坑，务必记牢）：调试时多次重启，
    每次用 `kill -9` 杀 `ros2 launch` 进程——但 SIGKILL **不会**把信号传给子节点，于是每次启动的
    每车辅助节点（`odom_sim_filter`/`battery_sim`/`charger_dock_monitor`/**`tf_relay`**）全部变孤儿残留。
    3 次重启后同时跑着 **3 整套** 机器人节点 + 6 个僵尸 `tf_relay`（各吃 ~25% CPU），
    多套 `tf_relay` 抢着给同名 `/tf` 重复发布 → **TF 混乱**，新 AMCL 建不起干净的 `map→odom`
    （日志刷 `Invalid frame ID "map"` / `Please set the initial pose`），同时 load 冲到 **46**。
    一度误判为"3 车算力不够"，实为进程泄漏。
    → **正确清理**：用 `setsid` 让 launch 自成进程组，停止时 `kill -9 -<PGID>` **整组**清掉；
    或正常用 **Ctrl-C**（`ros2 launch` 收到 SIGINT 会优雅关闭所有子节点，不会泄漏）。
    **切勿对 launch 用 `kill -9`**。另注意 `pkill -x <name>` 因 Linux comm 名截断到 15 字符
    （`component_container_isolated`→`component_conta`）会漏杀；`pgrep -f "<pattern>"` 又会匹配到
    含该 pattern 的自身命令行（计数虚高/自杀），排障时要 `grep -vw $$` 排除自身。

### 验证结果（headless，`gui:=false num_agvs:=3`，**干净机器**）
- **Composition 降耗**：每车恰好 **1 个 `component_container_isolated`**（3 车共 3 个，对比非 composition 的 21+ 进程）；
  3 车并发主动导航时 **load ≈ 6–8**（16 核），内存 ~5.5Gi/7.8Gi。对比僵尸态的 load 46 —— **降耗一个数量级**。
- **3 车并发端到端任务链全部跑通**（接近点修复后）：
  - `agv1`：`Assigned T1 → At pickup_A → Loaded → dropoff_A → Unloading → Task T1 complete` ✓
  - `agv3`：`Assigned T2 → At pickup_B → Loaded → dropoff_B → Unloading → Task T2 complete` ✓
  - （`agv2` 空闲：示例只定义了 T1/T2 两个任务。）
  - 全程 **planner 失败 0 / progress 失败 0 / 抢占 2（仅启动期）/ 停滞重发 3（仅启动期）**。
  - 3 车 `localization` + `navigation` lifecycle 均 `Managed nodes are active`。

### 已知限制 / 建议
- **3 车可正常运行**（干净启动下 load 6–8）；之前的过载是僵尸进程所致（问题 17），非 3 车本身。机器很弱时可 `num_agvs:=2` 进一步减压。
- 8GB WSL2 上 3 车内存 ~5.5Gi，尚有余量但不宽裕。
- 停止系统请用 **Ctrl-C**（或 `kill -9 -<PGID>` 整组），**勿对 launch 直接 `kill -9`**（见问题 17）。zsh 下务必 `source install/setup.zsh`。
- 接近点是"到站旁空地"，非压在托盘上——AGV 在托盘南侧 ~1.7m 处完成装卸（仿真足够，真实场景再加对接动作）。

## 11. Web 面板 UX 升级（对接 Nav2 后端）

把面板从"真值跟随器时代"升级到 Nav2，并对接本次后端新增的数据/接口。

### 本次后端更新的功能（面板所依赖）
- **`/fleet/state` 的 zones 增加 `gx,gy`**（导航接近点，见问题 15）：`fleet_manager.publish_fleet_state()`
  现在每个 zone 输出 `{cx,cy,sx,sy,gx,gy}`。已验证：`pickup_A` center=(5.5,5.5)→goal=(5.5,3.8)、
  `charger_1` center=(-5.5,-5.5)→goal=(-5.5,-4.8)，其余空地 zone `gx,gy==cx,cy`。
- **`fleet_manager` 改用接近点导航 + 停滞重发**（问题 15/16）：`nav_to_zone`/`at_zone` 走 `gx,gy`；
  重发改为停滞检测（不再盲发刷屏）。
- **launch 新增 `gui` 参数**（`warehouse_fleet_nav2.launch.py`），headless 压测用 `gui:=false`。
- 驱动层 = Nav2：每车发布 `/<ns>/plan`（`nav_msgs/Path` 全局规划）；`/<ns>/goal_pose`
  由 `nav2_goal_bridge` 消费 → `NavigateToPose`。

### 前端改动（`amr_web/web/{app.js,index.html,style.css}`）
- **Nav2 全局规划路径叠加**：每台车按需订阅 `/<ns>/plan`（`nav_msgs/Path`），在地图上画同色虚线路径
  —— 直观体现"真实路径规划"。
- **接近点可视化**：zones 的 `gx,gy` 与 `cx,cy` 不同时，画出菱形接近点标记 + 中心到接近点的连线，
  让操作员看清 AGV 实际停靠位置。
- **点击地图手动导航**：点击车队表某行选中一台车（高亮），再点击地图任意位置 → 发布
  `geometry_msgs/PoseStamped` 到 `/<ns>/goal_pose` 直接驱动 Nav2。**仅对 IDLE 车放行**
  （执行任务中的车会与调度目标冲突/抢占，故拦截并提示）；目标超出仓库范围也拦截。
- **运动可视化打磨**：AGV 尾迹（最近 50 点）、朝向用三角替代直线、选中车白色虚线高亮环、
  电量环 + 载货高亮保留。
- 图例新增"接近点 / Nav2 路径"，车队表行可点选（hover/选中样式）。
- 兼容性：仍只用 rosbridge + 本地内置 roslibjs；重连时清空订阅缓存避免引用已关闭连接。

### 验证
- `node --check app.js` 通过；面板 4 个静态资源经 `http.server` 全部 HTTP 200，新符号
  （`onMapClick`/`nav_msgs/Path`/`goal_pose`）已在服务内容中。
- 单独跑 `fleet_manager` 抓 `/fleet/state --field data`，确认 zones 携带正确 `gx,gy`（见上）。
- 浏览器实跑（路径叠加渲染、点击导航）需 GUI，留待带界面整体联调。

### 排障补记（运维）
18. **`ros2 run X & kill -9 $!` 仍会留孤儿**：`kill -9` 杀的是 `ros2 run` 包装进程，真正的节点是其子进程、
    会被托孤。清理用 `pkill -x <node_comm>`（注意 comm 截断 15 字符）或对包装进程发 **SIGINT**。
    （与问题 17 同源：SIGKILL 不传播给子进程。）

## 12. 路径冲突·充电优先·自动回充·碰撞检测（防撞强化）

围绕"多车实际运行会撞车 / 充电流程 / 路径冲突优先级"做的一轮强化与排障。

### 新增/改动
- **充电速度** `battery_sim.charge_per_second` 0.02→**0.10**（约 4s 充过阈值差，原来太慢看不出来）；
  怠速耗电 `drain_per_second_idle` 0.0002→**0.001**（电量可见变化）。
- **充电对接半径** `charger_dock_monitor.enter_radius` 0.8→**1.2**、exit 1.1→**1.5**。
  原因：充电接近点距充电中心 0.7m，加 Nav2 到点容差 0.25m，小车实际停在距中心 ~0.97m 处，
  >0.8m 旧半径 → 判定未对接 → battery_sim 不充电（fleet 以为在充、实则电量不升）。放大后可靠对接。
- **通用优先级让行（碰撞预防核心）** `fleet_manager.apply_traffic_priority()`：
  任意两车进入 `yield_radius`(1.6m) 时，**优先级低的一方原地停车让行**，对方驶离后恢复原目标。
  优先级：充电(TO_CHARGER/CHARGING)=5 > 送货(TO_DROPOFF/UNLOADING)=4 > 取货(TO_PICKUP/LOADING)=3 > 空闲=1；
  平级按 ns 字典序，仅一方让行避免死锁。**充电车享最高优先级**（用户要求：缺电车不能被堵）。
  让行中跳过常规重发、不参与任务分配；恢复时 `reissue_goal()` 按状态重下目标。
- **碰撞检测 + 告警 + 前端显示** `fleet_manager.check_collisions()`：两车中心距 < `collision_dist`(0.55m)
  即判危险接近，**边沿触发 ERROR 告警**(→Web 异常日志)，并在 `/fleet/state` 发布 `collisions`/`collision_count`。
  Web：地图上碰撞两车间画红线+红圈，标题红色"碰撞 N"角标，地图上方红色闪烁告警条。
- **导航就绪显示** `nav2_goal_bridge` 发 `/<ns>/nav_ready`(Nav2 激活前 False)；`fleet_manager` 汇总进
  `/fleet/state`；Web 车队表对未就绪车标红"导航未就绪"。（满足"导航状态未加载在 Web 显示"）

### 关键排障（启动健壮性 — 修 agv2/agv3 不动）
19. **Gazebo `/spawn_entity` 服务慢 → spawn 超时 → 小车不生成**：机器长时间运行/WSL 重启后，
    gazebo 加载世界(20货架+模型)>38s，`spawn_entity.py` 默认 30s 等待超时退出 → 三车不生成 →
    无 ground_truth → AMCL 无法定位（连锁全坏）。→ **修复**：spawn 加 `-timeout 120`、起始延后到 8/11/14s。
20. **Nav2 早于 spawn 启动 → 后启动的车导航激活失败**（agv2 典型不动）：原 Nav2 按固定定时器
    12/24/36s 启动，与被慢 gazebo 推迟的 spawn 脱钩，Nav2(AMCL/costmap) 在机器人还没生成时就激活→失败。
    → **修复**：改 **事件驱动**——`RegisterEventHandler(OnProcessExit(spawn))`，每车 **spawn 成功后**才启动其 Nav2。
21. **3 套 Nav2 容器同时加载组合节点 → `load_node` 超时**（agv3 的 bt_navigator/amcl 加载失败→导航未就绪）：
    composition 下三容器并发 `load_node` 会丢响应。→ **缓解**：on_exit 内按车号 **2/17/32s 大错峰**，
    每套 Nav2 间隔 ~15s 顺序加载。实测 agv1/agv2 稳定就绪；**agv3(第3套)在本机 WSL2 仍偶发某个组合节点
    (amcl) 加载失败** → 这是单机 8GB/16核 跑 3 套完整 Nav2 的资源/时序极限，非僵尸问题（新 WSL 仍现）。
    `nav_ready` 红标会如实暴露未就绪的车。**结论：2 车在本机完全可靠；3 车可用但第 3 套 Nav2 偶发未就绪。**

### 验证（headless）
- agv1/agv2：`nav_ready=True`、成功导航到各自待命点；3 车场景下 agv3 因上条偶发 `导航未就绪`。
- 碰撞检测：实测捕获 `agv1 与 agv2 危险接近 0.44m，累计 1 次`（功能正确）。
- 通用优先级让行 + 回充全流程（快充+可靠对接）：以 `num_agvs:=2` 复现验证（agv1 低电去充电享最高优先级，
  agv2 让行 → 碰撞预防；agv1 对接后电量按 0.10/s 回升至 0.60 → 回 IDLE）。
