# Design Spec: amr_qt_panel —— AMR 车队原生 QT 桌面面板

*   **Date**: 2026-06-21
*   **Status**: Draft
*   **Topic**: 新增 ROS 2 包 `amr_qt_panel`，用 PyQt5 做一个原生桌面面板，全量复刻现有 web 面板（`amr_web`），并以**直接订阅 ROS 2 图像话题**的方式提供稳定的 YOLO 标注视频流。

---

## 1. 背景与动机

现有 web 面板（`amr_web`）的相机视频通过 `web_video_server` 的 HTTP **快照轮询**获取（见 `app.js` 第 843 行起，`/<ns>/camera/image_raw` 的 snapshot 轮询）。这条链路在网络/浏览器侧不稳定，导致视频卡顿。

**根因**：视频走的是 HTTP 长轮询，而非 ROS 2 原生传输。

**解决思路**：做一个原生 QT 桌面应用，用 `rclpy` 直接订阅 `/{ns}/camera/image_raw`（本地 DDS），**完全绕开 `web_video_server`**，从根上消除 HTTP 轮询带来的卡顿；同时把 web 面板的地图、任务分配、车队/路权/异常/日志/下发任务等能力一并原生复刻。

## 2. 目标与非目标

### 2.1 目标
1. 新增独立 `ament_python` 包 `amr_qt_panel`，可 `colcon build` 并 `ros2 run` / `ros2 launch` 启动。
2. **全量对齐** web 面板的信息面板：实时地图、任务分配、车队状态、路权与走廊、系统异常、`/rosout` 日志、下发任务。
3. **YOLO 视频**：单路大画面 + 下拉切车，原生订阅图像话题，QT 内用 QPainter 叠加 YOLO 检测框。
4. **地图完整复刻**：占用栅格 + 货架/取货/卸货/充电/走廊/等待点 + Nav2 路径 + AGV 朝向三角，含滚轮缩放/拖动平移、点车→点图手动导航。
5. 视频稳定性显著优于现有 web 方案（不经 HTTP）。

### 2.2 非目标
- 不修改 `amr_vision`（含 `camera_detection_node.py`、`fleet_manager_ai.py`）。本面板只作为**消费端**。
- 不替换或删除 `amr_web`，两者并存。
- 不引入 `web_video_server` 依赖。
- 不要求 `cv_bridge` / `cv2` / `ultralytics`（面板侧）。

## 3. 架构

### 3.1 ROS ↔ Qt 对接（核心决策：方案 A）

PyQt5 与 rclpy 各有事件循环。采用**后台执行器线程 + Qt 信号**：

- `RosBridge(QObject)` 创建 rclpy `Node` 与 `SingleThreadedExecutor`，在独立 `threading.Thread` 中 `executor.spin()`。
- 所有订阅回调运行在该后台线程，**只做最小处理后 `emit` Qt 信号**；槽函数在 GUI 线程更新控件（Qt 跨线程信号为队列投递，线程安全）。
- 发布（goal_pose / add_task / 切换相机重订阅）由 GUI 线程经 `RosBridge` 的方法触发。
- 关闭时：`executor.shutdown()` → `join` 线程 → `node.destroy_node()` → `rclpy.shutdown()`。

> 备选：QTimer + `spin_once`（单线程，但 spin 与渲染抢 GUI 线程，图像一多会卡，正中痛点）；线程 + `queue.Queue`（Qt 信号已是跨线程队列，手搓队列更啰嗦）。均不采用。

### 3.2 线程边界（铁律）
- **只有**槽函数（GUI 线程）能触碰 QWidget。
- 后台线程只 `emit` 信号，不直接改控件。
- `sensor_msgs/Image → QImage` 的转换放在后台线程做，再 `emit`，避免 GUI 线程解码大图。

## 4. 包结构

```
src/amr_qt_panel/
  package.xml            # ament_python; exec_depend: rclpy std_msgs sensor_msgs
                         #   nav_msgs geometry_msgs rcl_interfaces python3-pyqt5
  setup.py               # console_scripts: amr_qt_panel = amr_qt_panel.app:main
  setup.cfg
  resource/amr_qt_panel
  amr_qt_panel/
    __init__.py
    app.py               # main(): rclpy.init → QApplication → RosBridge → MainWindow → exec
    ros_bridge.py        # RosBridge(QObject): node + 执行器线程 + 信号 + 发布器（唯一 ROS 边界）
    model/
      __init__.py
      fleet_state.py     # 解析 /fleet/state JSON → dataclass（单一数据源）
      coords.py          # 世界↔像素变换 + zoom/pan（移植 app.js）
      image_convert.py   # sensor_msgs/Image → QImage（numpy 手动转，无 cv2）
    widgets/
      __init__.py
      main_window.py     # QMainWindow：左地图 + 右 QScrollArea 信息栏，连信号→控件
      map_view.py        # MapView(QWidget)：QPainter 重画地图，点击导航，滚轮缩放
      camera_view.py     # CameraView(QWidget)：大画面 + 切车下拉 + YOLO 叠框
      task_table.py      # 任务分配
      fleet_table.py     # 车队状态（点行选车）
      row_table.py       # 路权与走廊
      anomaly_list.py    # 系统异常
      log_list.py        # /rosout 日志（含 INFO 开关 + 清空）
      task_form.py       # 下发任务（取货/卸货 → /fleet/add_task）
  launch/
    qt_panel.launch.py   # 启动 amr_qt_panel 节点（透传 map_ns 等参数）
  test/
    test_coords.py       # 坐标往返
    test_fleet_state.py  # JSON 解析
    test_image_convert.py# Image→QImage（offscreen）
```

## 5. 模块设计（每个单元：职责 / 接口 / 依赖）

### 5.1 `RosBridge(QObject)` —— 唯一 ROS 边界
**职责**：拥有 rclpy 节点、执行器线程、全部订阅/发布；把 ROS 数据转成 Qt 信号。

**Qt 信号（emit 给 GUI）**
| 信号 | 载荷 | 来源话题 |
|---|---|---|
| `fleet_state_changed(object)` | `FleetState` 模型 | `/fleet/state` |
| `map_changed(str, object)` | (ns, OccupancyGrid 数据) | `/{ns}/map` |
| `path_changed(str, object)` | (ns, `[(x,y), …]`) | `/{ns}/plan` |
| `image_changed(str, object)` | (ns, `QImage`) | `/{ns}/camera/image_raw` |
| `detections_changed(str, object)` | (ns, `[bbox…]`) | `/{ns}/camera/detections` |
| `log_received(object)` | 日志条目 | `/rosout` |

**方法（GUI 调用）**
- `publish_goal(ns, x, y, yaw=0.0)` → 发 `PoseStamped` 到 `/{ns}/goal_pose`
- `publish_add_task(pickup, dropoff)` → 发 JSON 到 `/fleet/add_task`
- `set_active_camera(ns)` → 销毁旧的 image/detections 订阅，按新 ns 重建（单路相机省资源）
- `set_map_ns(ns)` → 切换订阅哪台车的 `/map`（默认 `agv1`，可由 launch 参数 `map_ns` 覆盖）
- `shutdown()` → 优雅停止线程与 rclpy

**依赖**：rclpy、各 msg 包、`model/*`。

### 5.2 `model/fleet_state.py`
**职责**：把 `/fleet/state` JSON（见 §6.2）解析成 dataclass，作为各表的单一数据源。容错：字段缺失给默认值，不抛异常。
**接口**：`FleetState.from_json(str) -> FleetState`，含 `agvs / tasks / zones / anomalies / segments / wait_points / queued_tasks / idle_agvs / collisions …`。

### 5.3 `model/coords.py`
**职责**：世界坐标 ↔ 画布像素，含 zoom/pan。**逐行移植 `amr_web/web/app.js` 第 127–148 行**的 `VIEW/PAD/SPAN/SIZE`、`baseX/baseY/baseL`、`toX/toY/toL`、逆变换 `fromX/fromY`。
**接口**：`Transform(size).to_px(wx,wy) / to_len(m) / from_px(px,py)`；`zoom/pan` 为可变状态。
**测试**：`from_px(to_px(p)) ≈ p` 往返一致。

### 5.4 `model/image_convert.py`
**职责**：`sensor_msgs/Image → QImage`，**纯 numpy，无 cv2**：
- `rgb8` → `QImage.Format_RGB888`
- `bgr8` → `Format_BGR888`（Qt ≥5.14）或 `rgbSwapped()` 回退
- `mono8` → `Format_Grayscale8`
- 必须 `.copy()`（QImage 不持有 numpy 内存）
**依赖**：numpy（ROS 2 已自带）。**理由**：绕开仓库已记录的 cv_bridge NumPy ABI 问题。

### 5.5 `widgets/map_view.py`
**职责**：`paintEvent` 中按图层重画（移植 app.js `drawStatic/drawMap/drawPaths/drawAgvs` 逻辑）：
1. 背景填充；2. 占用栅格（新栅格到达时构建一次 QImage 缓存，`imageSmoothingEnabled=false` 等价：关插值）；3. 静态图元（zones：货架/取货/卸货/充电、接近点 gx/gy、走廊段、等待点）；4. Nav2 路径（虚线，按车着色）；5. AGV 朝向三角 + 电量/状态色 + 标签 + 碰撞高亮。
**交互**：滚轮缩放（改 `zoom`）、拖动平移（改 `pan`）、点击：若已选车则 `from_px` → `publish_goal`。
**输入**：`fleet_state_changed`（AGV/zones/segments）、`map_changed`、`path_changed`。

### 5.6 `widgets/camera_view.py`
**职责**：显示当前选中车的图像（`KeepAspectRatio` 缩放），叠加 YOLO 框（来自 `detections_changed` 的像素 bbox，按缩放比换算到控件坐标，标注 class+conf）。顶部下拉切车 → `RosBridge.set_active_camera(ns)`。
**容错**：无图像时显示「等待相机话题…」。

### 5.7 表格/列表/表单（对齐 web）
- `task_table`：任务编号 / 小车编号 / 运行状态 / 任务内容（来自 `tasks[]`）。
- `fleet_table`：AGV / 状态 / 电量 / 载货 / 位置 / 待命点（`agvs[]`）；**点行选车**用于手动导航。
- `row_table`：走廊段 / 占用车 / 等待队列 / 位置范围（`segment_*` + `corridor_segments`）。
- `anomaly_list`：系统异常（`anomalies[]`，按 level 着色）。
- `task_form`：取货/卸货下拉（由 `zones` 填充，充电区 `charger_zones` 排除）→ `publish_add_task`；显示 `queued_tasks`。
- `log_list`：`/rosout`，INFO 开关 + 清空（对齐 app.js 第 1131 行起逻辑）。

## 6. 数据契约

### 6.1 订阅与 QoS（关键）
| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/fleet/state` | `std_msgs/String` | 默认 reliable, keep_last 10 | ~3 Hz 全量快照 |
| `/{ns}/map` | `nav_msgs/OccupancyGrid` | **RELIABLE + TRANSIENT_LOCAL, depth 1** | 不配 transient_local 收不到 latched 地图（地图空白） |
| `/{ns}/plan` | `nav_msgs/Path` | 默认 reliable | Nav2 路径 |
| `/{ns}/camera/image_raw` | `sensor_msgs/Image` | **`qos_profile_sensor_data`（BEST_EFFORT）** | 不配 best-effort 可能收不到帧 |
| `/{ns}/camera/detections` | `std_msgs/String` | 默认 reliable | YOLO bbox JSON |
| `/rosout` | `rcl_interfaces/msg/Log` | 默认 | 日志 |

### 6.2 `/fleet/state` JSON（取自 `fleet_manager_ai.publish_fleet_state`，权威）
顶层键：`stamp, world_name, agvs[], queued_tasks, zones{}, charger_zones[], zone_owner{}, segment_owner{}, segment_occupants{}, segment_dir{}, segment_queue{}, corridor_segments{}, wait_points{}, tasks[], idle_agvs[], yielding[], collisions[], collision_count, anomalies[], require_nav_ready`。
- `agvs[]`：`ns, x, y, yaw, battery, state, task, carrying, on_charger, nav_ready, stuck, home_x, home_y, semantic, semantic_stop`
- `tasks[]`：`id, pickup, dropoff, status('assigned'|'queued'), agv, agv_state`
- `zones{name}`：`cx, cy, sx, sy, gx, gy`
- `anomalies[]`：`level('error'|'warn'|'info'), ns, type, msg`

### 6.3 `/{ns}/camera/detections` JSON（取自 `camera_detection_node`）
`{timestamp, detections:[{class_name, confidence, bbox:[x1,y1,x2,y2]}], inference_ms, frame_id}`，bbox 为源图像像素坐标。

### 6.4 发布
- `/{ns}/goal_pose`（`geometry_msgs/PoseStamped`，`frame_id='map'`）—— 点击导航。
- `/fleet/add_task`（`std_msgs/String`，JSON `{"pickup":<zone>,"dropoff":<zone>}`）—— 下发任务；`zone` 必须是 `/fleet/state.zones` 的键。

## 7. 界面布局

```
┌──────────────────────────┬─────────────────────────┐
│                          │ 相机视频流 [agv1 ▼]     │
│   仓库实时地图            │  ┌───────────────────┐  │
│   (QPainter)             │  │  大画面 + YOLO框   │  │
│   栅格/货架/走廊          │  └───────────────────┘  │
│   Nav2路径/AGV▲          │ 系统异常                │
│   滚轮缩放·拖动平移       │ 下发任务 [取货▼][卸货▼] │
│   点车→点图=手动导航      │ 任务分配 (表)           │
│                          │ 车队状态 (表, 点行选车) │
│                          │ 路权与走廊 (表)         │
│                          │ 异常日志 /rosout        │
└──────────────────────────┴─────────────────────────┘
        左：MapView              右：QScrollArea 装全部面板
```

## 8. 错误处理与生命周期

- 各面板在数据未到时显示占位（「等待…」），缺话题不崩。
- `/fleet/state` 解析失败 → 跳过该帧并记内部告警，不影响 UI。
- 切车时旧 image/detections 订阅必须销毁，避免泄漏与多路解码。
- 窗口关闭 → `RosBridge.shutdown()` 有序停线程。
- PyQt5 未安装 → `app.py` 启动即给出清晰提示（`sudo apt install python3-pyqt5`）。

## 9. 测试策略

- **纯逻辑单测（无显示器）**：
  - `test_coords.py`：`from_px(to_px(p)) ≈ p` 往返；缩放/平移后仍一致。
  - `test_fleet_state.py`：喂 §6.2 样例 JSON，断言 dataclass 字段；缺字段容错。
  - `test_image_convert.py`：构造 rgb8/bgr8/mono8 numpy 图 → 断言 QImage 尺寸/格式（`QT_QPA_PLATFORM=offscreen`）。
- **集成验收**：连真实 `amr_vision_fleet` 仿真运行，确认视频流畅、地图复刻、各表数据、点击导航、下发任务生效。遵循已有「仿真启停协议」，**不要 `kill -9` launch**。

## 10. 风险与缓解

| 风险 | 缓解 |
|---|---|
| `/map` QoS 配错 → 地图空白 | §6.1 钉死 RELIABLE+TRANSIENT_LOCAL |
| image QoS 不匹配 → 收不到帧 | 用 `qos_profile_sensor_data` |
| 坐标系不一致（±7m 仓库 vs map.world ±20m） | `coords.py` 逐行移植 app.js 常量，集成期与 web 对照 |
| 全量复刻工作量大 | §11 分阶段，每阶段可独立验证 |
| 跨线程误触控件 | §3.2 铁律 + code review 检查 |

## 11. 分阶段实现概览（细节留给实现计划）

1. **阶段一**：包骨架 + `RosBridge`（线程/信号/优雅退出）+ `camera_view`（订 image_raw + detections，叠框，切车）→ 先交付「稳定视频」这一核心价值。
2. **阶段二**：`map_view` 完整复刻（coords 移植、栅格、静态图元、路径、AGV、缩放、点击导航）。
3. **阶段三**：各表/表单/日志（`fleet_state` 模型驱动）+ `main_window` 总装 + launch + 单测。

## 12. 验收标准

- `colcon build --packages-select amr_qt_panel` 通过；`ros2 launch amr_qt_panel qt_panel.launch.py` 起窗口。
- 仿真在跑时：视频比 web 明显更稳；地图、任务分配、车队、路权、异常、日志均有数据且与 web 一致；切车、点击导航、下发任务可用。
- 纯逻辑单测全绿。
