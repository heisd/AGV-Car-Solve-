# amr_vision Web 面板 + 相机视频流接入

> 把 Web 操作面板与 YOLO 相机视频流接入 `amr_vision_fleet.launch.py`，并将 `/fleet/state` 遥测 + `/fleet/add_task` 派单 + 走廊路权完整移植进 `fleet_manager_ai`
> 更新日期：2026-06-20

---

## 目录

1. [概述](#1-概述)
2. [架构与数据流](#2-架构与数据流)
3. [改动清单](#3-改动清单)
4. [相机话题修复（根因 Bug）](#4-相机话题修复根因-bug)
5. [启动与查看](#5-启动与查看)
6. [端到端验证记录](#6-端到端验证记录)
7. [已知限制 / Caveats](#7-已知限制--caveats)
8. [文件地图](#8-文件地图)

---

## 1. 概述

本次改动让 `amr_vision` 包**自带**完整的运营闭环，无需依赖 `amr_description` 的 fleet/web 栈：

| 能力 | 改动前 | 改动后 |
|------|--------|--------|
| Web 面板 | 需单独起 `web_panel.launch.py` | `amr_vision_fleet.launch.py` 一键带起（rosbridge + 静态 HTTP + web_video_server） |
| 车队遥测 `/fleet/state` | `fleet_manager_ai` 无此话题，面板空白 | 完整移植，3 Hz 发布，含语义字段 |
| 在线派单 `/fleet/add_task` | 无 | 已移植，面板可下发任务 |
| 走廊路权 / 区域预约 / 碰撞检测 | 仅在 `amr_description/fleet_manager.py` | 完整移植进 `fleet_manager_ai` |
| 相机视频流 | 面板无 | 每车一块 MJPEG tile（参考 LabRobot 面板实现） |
| YOLO 语义层 | 仅 `fleet_manager_ai` 内部 | 保留，并在面板相机 tile 上以徽标展示 |

设计取舍：遵循 `amr_vision` 的**自包含/复制**约定 —— `fleet_manager_ai.py` 不再是裁剪版重写，而是 `FleetManager` 的完整对等实现（FSM + 路权 + 区域预约 + 碰撞 + `/fleet/state` + `/fleet/add_task`）**叠加**既有 YOLO 语义层。

---

## 2. 架构与数据流

```
Gazebo (gzserver, libgazebo_ros_camera.so)
   └─ /<ns>/camera/image_raw  ──┬──► camera_detection_node  ─► /<ns>/camera/obstacle_semantic (YOLO 语义)
   (640×480 @ ~10Hz)            │                                         │
                               │                                         ▼
                               │                              fleet_manager_ai ─► /fleet/state (3Hz, 含 semantic)
                               │                                         │            ▲
                               ▼                                         │            │ /fleet/add_task
                       web_video_server :8082                            │       rosbridge :9090 (ws)
                  /stream?topic=/<ns>/camera/image_raw&type=mjpeg        │            │
                               │                                         ▼            ▼
                               └──────────────► 浏览器面板 (静态 HTTP :8080, app.js / roslibjs)
                                                  · 相机 tile <img>  · 车队表/地图  · 语义徽标
```

关键话题约定（**契约**，前端与 YOLO 节点都依赖）：

| 话题 | 类型 | 生产者 | 消费者 |
|------|------|--------|--------|
| `/<ns>/camera/image_raw` | `sensor_msgs/Image` | Gazebo 相机插件 | camera_detection_node、web_video_server |
| `/<ns>/camera/obstacle_semantic` | `std_msgs/String` | camera_detection_node | fleet_manager_ai |
| `/fleet/state` | `std_msgs/String` (JSON) | fleet_manager_ai | 面板 (rosbridge) |
| `/fleet/add_task` | `std_msgs/String` (JSON) | 面板 | fleet_manager_ai |

---

## 3. 改动清单

### 3.1 `src/amr_vision/launch/amr_vision_fleet.launch.py`
- `launch_setup` 内新增 Web 区块（约 L93–L153）：当 `launch_web=true` 时
  - include `rosbridge_server/launch/rosbridge_websocket_launch.xml`（端口 `rosbridge_port`）
  - `ExecuteProcess` 起 `python3 -m http.server <web_port>`（serve `amr_web/web`）
  - **web_video_server 受包存在性保护**（L130–L148）：`try: get_package_share_directory('web_video_server')` 成功才加 `Node`，否则打印 `LogInfo` 提示安装，**不再因缺包而整条 launch 崩溃**
- 新增启动参数（L435–L460）：`launch_web`(true)、`rosbridge_port`(9090)、`web_port`(8080)、`web_address`(0.0.0.0)、`video_port`(8082)

### 3.2 `src/amr_vision/amr_vision/fleet_manager_ai.py`（完整移植）
- 由 `amr_description/fleet_manager.py` 逐函数移植：`cb_odom`/`cb_plan`/`cb_nav_ready`、`apply_traffic_rules`、`check_collisions`、走廊路权全套（`detect_segments`/`_can_admit`/`_enter_yield`/`nearest_wait_point` 等）、`assign_tasks`（含区域预约）、`run_agv`（含返航 home）、`cb_add_task`、`publish_fleet_state`
- 叠加既有 YOLO 语义层：`_semantic_callback` / `_handle_semantic_change`（person→停车、pallet→减速、clear→恢复）/ `_cancel_nav_goal`；`reissue_goal` 在 `semantic_stop_active` 时不重发目标
- `publish_fleet_state` 每车新增 `semantic` / `semantic_stop` 字段；异常含「行人停车」「托盘减速」

### 3.3 `src/amr_vision/urdf/amr_vision.urdf.xacro`
- 相机插件 remap 修复（见 §4）

### 3.4 Web 前端（`src/amr_web/web/`）
- `index.html`：`.side` 顶部新增「相机视频流」面板（`#camPort`=8082、`#camQuality`=60、`#camReload`、`#cameraGrid`）
- `app.js`：相机模块 —— `videoBase()` / `camStreamUrl(ns)`（`/stream?topic=/<ns>/camera/image_raw&type=mjpeg&quality=..`，L810）/ `renderCameras(agvs)`（按 `ns` 集合幂等重建，徽标读 `semantic`/`semantic_stop`，L829）；接入 `/fleet/state` 回调
- `style.css`：`.cam-grid` / `.cam-tile` / `.cam-img` / `.cam-sem`（`.sem-clear`/`.sem-pallet`/`.sem-person`）/ `.cam-err`

### 3.5 `src/amr_vision/package.xml`
- 新增 `exec_depend`：`amr_web`、`rosbridge_server`、`web_video_server`

---

## 4. 相机话题修复（根因 Bug）

**症状**：面板相机 tile 与 YOLO 节点都收不到帧。

**根因**：`libgazebo_ros_camera.so` 发布的话题以 `<camera name>` 为前缀，即 `/<ns>/<ns>_camera/image_raw`。原 remap 的「from」写成裸 `image_raw`，匹配不上，导致帧发到 `/agv1/agv1_camera/image_raw`，而 `camera_detection_node` **和**前端都订阅规范话题 `/agv1/camera/image_raw`（发布者数=0）。

**修复**（`amr_vision.urdf.xacro:240-241`）：remap 的「from」改为带相机名前缀的真实相对名：
```diff
- <remapping>image_raw:=camera/image_raw</remapping>
- <remapping>camera_info:=camera/camera_info</remapping>
+ <remapping>${ns}_camera/image_raw:=camera/image_raw</remapping>
+ <remapping>${ns}_camera/camera_info:=camera/camera_info</remapping>
```
`xacro` 展开（`namespace:=agv1`）后为 `agv1_camera/image_raw:=camera/image_raw`，最终落到规范话题 `/agv1/camera/image_raw`。

> colcon 使用 `--symlink-install`，`install→build→src` 三级软链均指向源文件，**改 URDF 无需 rebuild**，重启仿真即生效。

---

## 5. 启动与查看

```zsh
# 一键起：Gazebo + 单车 + Nav2 + 相机 + Web 全栈（standalone）
source /opt/ros/humble/setup.zsh && source install/setup.zsh
ros2 launch amr_vision amr_vision_fleet.launch.py launch_gazebo:=true num_agvs:=1
```

浏览器打开 **http://localhost:8080** —— 左侧「相机视频流」面板显示每车实时画面。

常用参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `launch_gazebo` | `false` | standalone（自起 Gazebo+全节点）；false 时假定显示/spawn launch 已在运行 |
| `num_agvs` | `1` | 车队数量 |
| `launch_web` | `true` | 带起 rosbridge + 静态 HTTP + web_video_server |
| `rosbridge_port` / `web_port` / `video_port` | `9090` / `8080` / `8082` | 三个 Web 端口 |
| `web_address` | `0.0.0.0` | HTTP 绑定地址（0.0.0.0 = 局域网可达） |

**停止**（遵循 run/stop 规范，勿对 `ros2 launch` PID 直接 `kill -9`）：前台用 `Ctrl-C`；后台用 `setsid` 起、再 `kill -INT -<PGID>`（必要时 `kill -9 -<PGID>`）整组杀。

---

## 6. 端到端验证记录

环境：WSL2 + ROS2 Humble + Gazebo Classic；`launch_gazebo:=true num_agvs:=1`；headless（gzserver 渲染相机）。日期 2026-06-20。

| 链路 | 检查命令 / 方式 | 结果 |
|------|----------------|------|
| 相机渲染真实场景 | `curl .../snapshot` → 查看 JPEG | 640×480 货架场景，~8KB ✅ |
| 规范相机话题 | `ros2 topic info /agv1/camera/image_raw` | 修复后 Publisher=1 / Subscriber=1，~2–6 Hz ✅ |
| YOLO 节点订阅 | 同上 Subscriber=1（camera_detection_node） | ✅ 收到帧 |
| web_video_server MJPEG | `curl '.../stream?topic=/agv1/camera/image_raw&type=mjpeg'`（前端实际 URL） | `multipart/x-mixed-replace`，**5s 50 帧** ✅ |
| web_video_server 快照 | `curl '.../snapshot?...'` | HTTP 200，合法 JPEG（ffd8ff） ✅ |
| rosbridge 遥测 | `ros2 topic echo --full-length /fleet/state` | 3 Hz，含 `semantic="clear"`/`semantic_stop=false` ✅ |
| 面板静态页 | `curl http://localhost:8080/index.html` | HTTP 200 ✅ |
| 前端 URL 契约 | 审 `app.js:camStreamUrl()` | 生成 URL 与实测一致；tile 按 `ns` 索引；徽标读 `semantic` ✅ |
| YOLO 推理运行（修 NumPy 后） | `ros2 topic echo /agv1/camera/detections` | `YOLO model loaded`，`inference_ms≈78`，`obstacle_semantic` ~1–2 Hz ✅（空场景语义=clear） |

**结论**：视频流可在前端正常加载；YOLO 推理在修复 NumPy ABI 冲突后已实际运行。

---

## 7. 已知限制 / Caveats

1. **YOLO 推理 NumPy ABI 冲突（已修复，2026-06-20）**：`ultralytics` / `torch` / `opencv-python` / `cv-bridge` 本就已安装，缺包不是原因。真正的坑是**双向 NumPy ABI 冲突**：
   - `cv_bridge.boost`（ROS2 Humble 编译）针对 **NumPy 1.x** —— 在 NumPy 2.x 下 `from cv_bridge import CvBridge` 抛 `AttributeError: _ARRAY_API not found`；
   - 但 `opencv-python 4.12` 又是针对 **NumPy 2.x** 编译的（`requires numpy>=2`）。

   两者对 NumPy 大版本要求相反，必须把**两个**都钉到 NumPy 1.x 一侧才一致：
   ```zsh
   pip install "numpy<2" "opencv-python<4.10"
   # 实测落到: numpy 1.26.4 + opencv-python 4.9.0.80
   # torch 2.11 / ultralytics 8.4 均兼容 numpy 1.26
   ```
   未修复时 `_CV_BRIDGE_AVAILABLE=False` → `_image_callback` 在推理前 return → `obstacle_semantic` 不发布、面板徽标恒 `clear`（视频流本身不受影响）。修复并重启后已验证：`YOLO model loaded: yolov8n.pt`、`/<ns>/camera/detections` 的 `inference_ms≈78ms`、`obstacle_semantic` 正常发布（空场景下语义为 `clear`，属正常）。
   > 注：YOLO 权重 `yolov8n.pt` 已缓存于工作区根目录（首次运行会联网下载 ~6MB）。空仓库场景没有 person/suitcase，徽标维持 `clear` 是**正确**结果；要看到 `person·停车`/`pallet·减速` 需在相机前放置可检测目标。

2. **web_video_server 为软依赖**：未安装时 launch 不崩溃，仅打印提示、相机流禁用。安装：
   ```zsh
   sudo apt install -y ros-humble-web-video-server
   ```

3. **走廊路权坐标系**：`CORRIDOR_SEGMENTS` / `WAIT_POINTS` 系从 `amr_description` 的 ±7m 地图逐字复制，在 `amr_vision` 的 ±20m 地图上坐标**不对位**（路权逻辑可运行但分段不命中实际走廊）。源码头部已加 NOTE 标注；如需在 ±20m 地图启用，需按新地图重标定坐标。

4. **WSL2 MJPEG 长连接**：偶发卡帧；直连 `video_port` 一般可用，必要时可改快照轮询（`app.js` 已留注释）。

---

## 8. 文件地图

| 文件 | 角色 |
|------|------|
| `src/amr_vision/launch/amr_vision_fleet.launch.py` | 全栈 launch（含 Web 区块 + 参数） |
| `src/amr_vision/amr_vision/fleet_manager_ai.py` | 车队 FSM + 路权 + 遥测 + 派单 + YOLO 语义 |
| `src/amr_vision/amr_vision/camera_detection_node.py` | YOLOv8 相机感知（detections + obstacle_semantic） |
| `src/amr_vision/urdf/amr_vision.urdf.xacro` | 机器人模型（相机插件 remap 在此） |
| `src/amr_web/web/{index.html,app.js,style.css}` | 操作面板前端（含相机 tile 模块） |
| `src/amr_vision/package.xml` | 依赖声明（amr_web / rosbridge_server / web_video_server） |
