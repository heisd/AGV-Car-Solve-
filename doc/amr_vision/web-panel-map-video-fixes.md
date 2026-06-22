# Web 面板：视频流 / 地图自适应 / 区域划分 / 鼠标缩放

> 在 [web-camera-integration](web-camera-integration.md) 基础上，修复面板视频流不出图、地图错位，并实现「按地图的区域划分」与地图鼠标缩放
> 更新日期：2026-06-20

---

## 目录

1. [概述](#1-概述)
2. [视频流不出图：%2F 编码 Bug](#2-视频流不出图2f-编码-bug)
3. [地图错位：VIEW 自适应真实地图](#3-地图错位view-自适应真实地图)
4. [取货/卸货/充电点：数据驱动按世界显示](#4-取货卸货充电点数据驱动按世界显示)
5. [区域划分：井字主干道（后端发布几何）](#5-区域划分井字主干道后端发布几何)
6. [地图鼠标缩放 / 平移](#6-地图鼠标缩放--平移)
7. [改动文件清单](#7-改动文件清单)
8. [验证记录](#8-验证记录)
9. [已知限制 / 后续](#9-已知限制--后续)
10. [前端代码验证规范](#10-前端代码验证规范)

---

## 1. 概述

前一阶段接入了 Web 面板与相机流后，浏览器实测暴露出 4 个问题，本轮全部解决：

| 现象 | 根因 | 修复 |
|------|------|------|
| 相机面板「无视频流」 | 前端把话题 `/` 编码成 `%2F`，web_video_server 不解码 | `camStreamUrl` 保留原始斜杠 |
| 地图只显示一角、错位 | `VIEW` 固定 ±7.6 m，真实地图是 ±25×±15 m | 收到 `/map` 时自动按地图范围定标 `VIEW` |
| 取货/充电点位置不对 | 画的是硬编码 ±7 仓库示意 | 改为从 `/fleet/state.zones` 按类型着色绘制 |
| 区域划分（走廊）不对/隐藏 | 走廊几何硬编码 ±7，且未随地图发布 | 后端发布井字主干道几何，前端按图绘制 |
| （新增）地图无法缩放查看 | 无缩放能力 | 滚轮缩放 + 拖动平移 |

核心思路：**前端尽量数据驱动**——地图尺度取自 `/map`，分区与走廊几何取自 `/fleet/state`，从而「不同地图自带不同显示」。

---

## 2. 视频流不出图：%2F 编码 Bug 与高频销毁重建 Bug

**症状**：相机面板显示「⚠ 无视频流」，但从 WSL 用 curl 取流正常。

### 2.1 %2F 编码 Bug
**根因**：`app.js` 用 `URLSearchParams` 拼 URL，会把话题里的 `/` 百分号编码成 `%2F`。该版本 `web_video_server` **不解码** `%2F`，于是报错并拒绝订阅：
```
web_video_server: Invalid topic name: '%2Fagv1%2Fcamera%2Fimage_raw'
```
浏览器拿到的是「200 + multipart 头但零帧」，`<img>` 加载失败 → 显示无视频流。curl 因为用的是字面 `/` 才正常。

**修复**（`app.js: camStreamUrl`）：不用 `URLSearchParams`，话题斜杠保持原样：
```js
const topic = `/${ns}/camera/image_raw`;
return `${videoBase()}/stream?topic=${topic}&type=mjpeg&quality=${q}&_=${Date.now()}`;
```

**证据**：编码形式 `%2F` → **0 帧**；字面 `/` → **39 帧 / 4s**（`multipart/x-mixed-replace`）。

### 2.2 高频销毁重建导致流请求被中止（Abort）的 Bug
**根因**：后端以高频发布车辆遥测状态（`/fleet/state`），但数据帧中的 `state.agvs` 可能会因为网络时延或短暂的状态未就绪在某些帧发生变空等抖动。前端 `renderCameras` 原本使用的是直接重写 `cameraGrid.innerHTML` 的方式来动态重建车辆相机画面。这导致 `state.agvs` 只要发生一帧变化，已建的 `<img>` 标签就会从 DOM 中被彻底移除，从而强制触发浏览器对视频流长连接的 `Abort`（中止），并于下个周期重新建流。这种不到 1 秒的高频断开与重连使浏览器完全来不及解码第一帧，使得画面持续卡在灰色报错状态。

**修复**（`app.js: renderCameras`）：
- 废除原先暴力的 `innerHTML` 整体重置方案，改为**增量式 DOM 更新（Incremental DOM Update）**。
- 动态维护现有的 `.cam-tile`。每次仅将新加入的车辆增量插入 DOM 并单独启动其视频流；当车辆下线时才将其移出；若列表短暂变空时保持现状以防闪断。这保证了已连接的视频流长连接不被干扰和销毁。

---

## 3. 地图错位：VIEW 自适应真实地图

`my_map`（`maps/my_map.yaml`）：`resolution 0.05`、`998×598`、`origin (-25,-15)` → 世界范围 **x∈[-25,25]、y∈[-15,15]**（±25×±15 大仓）。

面板原本 `VIEW=7.6`（±7.6 m），把世界 [-7.6,7.6] 映射到画布。于是真实地图、分区、AGV 都被画到画布外，只剩硬编码的 ±7 示意图可见 → 严重错位。

**修复**（`app.js: onMapMsg`）：收到 `/map` 时按地图范围定标 `VIEW`（地图以原点为中心，取较大半边长 + 4% 余量），并置 `mapViewLocked=true`，避免 `world_name` 切换覆盖：
```js
const halfX = Math.max(Math.abs(originX), Math.abs(originX + w*res));
const halfY = Math.max(Math.abs(originY), Math.abs(originY + h*res));
VIEW = Math.max(halfX, halfY, 1) * 1.04;   // 本图 ≈ 26
```
> 假设地图以世界原点为中心（amr_description ±7 与 amr_vision ±25 均满足）。非居中地图需额外引入中心偏移。

地图点击下发目标的范围限制也同步改为按真实地图边界（x/y 分别判断，地图可能非方形）。

### 3.1 非硬编码大仓世界关闭/无真实地图时的回退错位

**症状**：如果网页端“真实地图(/map)”未勾选，或者未收到真实地图，渲染会回退到硬编码示意图。但在大仓世界下，地图尺度 `VIEW` 和分区、AGV 的坐标已经自适应为大仓尺度（±25 m），而回退渲染机制会画出默认小仓（±7 m）的地面、外墙和货架，导致大仓的真实分区被画到了“外墙”外面，且正中央出现了一排实际上并不存在的“幽灵货架”。

**修复**：
- **默认地图初始化**：在 `app.js` 中将初始默认的 `currentWorldName` 设为 `'map'`（空模板），防止在页面刚加载、rosbridge 尚未连上或尚未收到首条 `/fleet/state` 遥测消息的“初始空白期”时，因默认引用小仓 `'warehouse'` 导致强行绘制出旧的小仓外墙与货架。
- **动态注册未知场景**：在接收到车队状态消息时，动态注册未知场景模板，将 `currentWorldName` 设为 `state.world_name`，防止对未知的大仓世界（如 `my_map`）依然引用默认的 `warehouse` 配置；
- **回退渲染类型守卫**：在 `drawStatic` 中加入场景类型守卫，仅对已定义硬编码布局的已知小仓场景（如 `warehouse`、`complex_warehouse`、`warehouse_complex`）才绘制回退的地面、外墙和货架。对于大仓场景等未硬编码布局的未知场景，在关闭或无真实地图时只填充深色底色，从而保证走廊、分区和 AGV 不会出现错位绘制。


---

## 4. 取货/卸货/充电点：数据驱动按世界显示

`/fleet/state` 已经发布每个世界的真实分区坐标（后端从世界的 zones 加载），例如本图：
```
pickup_zone_A(-12,-6)  pickup_zone_B(12,-6)
dropoff_zone_A(-15,10) dropoff_zone_B(15,10)
charger_1(-20,-10)     charger_2(20,-10)
```

**修复**（`app.js: drawStatic`）：
- 不再绘制硬编码的 ±7 `WAREHOUSE.pickup/charger`（仅当无 `/fleet/state.zones` 时回退）；
- 改为遍历 `/fleet/state.zones`，按名称前缀**分类着色**：取货=绿、卸货=蓝、充电=橙，填充淡色块 + 描边 + 标签；被某车预约时用该车颜色高亮。

这样切换世界即自动显示该世界的正确取货/卸货/充电点。

---

## 5. 区域划分：井字主干道（后端发布几何）

本图是开阔大仓，无物理窄通道；上方两排长货架（左 x≈[-18,-6]、右 x≈[2,14]，y≈8）挡在取货(y=-6)与卸货(y=10)之间，纵向只能走**中央间隙**与**左右外侧**。据此设计**井字主干道**（3 纵 + 2 横，避开货架）：

```
 D═══════╦═══════D    北横道 y[10,13]（卸货线，货架上方）
 ║ [货架]║ [货架]║
 ║       ║       ║    西/中/东 三纵道（避开货架的可通行列）
 P       ║       P
 C═══════╩═══════C    南横道 y[-13,-9]（取货/充电线）
```

**后端**（`fleet_manager_ai.py`）：
- `CORRIDOR_SEGMENTS` 改为井字几何，`WAIT_POINTS` 改为 8 个交叉口让行点：
  ```
  west  x[-24,-19]   center x[-4,1]    east x[15,20]    （三纵，y[-13,13]）
  north y[10,13]     south  y[-13,-9]                   （两横，x[-24,20]）
  ```
- `publish_fleet_state` 新增发布几何：`corridor_segments`、`wait_points`。

**前端**（`app.js`）：`drawCorridors` / `drawWaitPoints` / 路权表 `renderRow` 全部改为**优先读 `/fleet/state.corridor_segments` / `wait_points`**（无则回退硬编码，且尺度不符时由 `hardcodedOverlaysMatchMap` 隐藏）。占用/预约/排队状态仍由 `segment_owner`/`segment_queue` 着色。

效果（叠加到真实地图自验证）：三纵道分别落在左货架外侧 / 中央间隙 / 右货架外侧，两横道连通卸货区与充电区，等待点在各交叉口，布局合理、避开货架。

---

## 6. 地图鼠标缩放 / 平移

`app.js` 在基础变换上叠加 `zoom`/`panX`/`panY`：
```js
const toX = (wx) => baseX(wx) * zoom + panX;   // baseX/baseY/baseL 为 VIEW 基础变换
```
- **滚轮缩放**：围绕光标 1×–8×（保持光标处世界点不动），缩回 1× 自动复位 pan；
- **平移**：放大后按住拖动；未放大时不平移以保留点击下发目标；
- 拖动超过阈值会置 `_dragMoved`，**抑制紧随的 click**，避免拖动误下发目标；
- `fromX/fromY`（像素→世界，供点击换算）已计入 zoom/pan。

---

## 7. 改动文件清单

| 文件 | 改动 |
|------|------|
| `src/amr_web/web/app.js` | camStreamUrl 去 `%2F`；onMapMsg 自适应 VIEW + `mapViewLocked`；world_name 守卫；drawStatic 分区按类型着色、去硬编码取货/充电；drawCorridors/drawWaitPoints/renderRow 读后端几何；toX/toY 缩放变换 + 滚轮/拖动事件；onMapClick 范围修正 + 防拖动；drawStatic 中对非硬编码场景（如大仓）跳过画示意墙/货架以防止关图错位；动态注册未知场景模板；currentWorldName 默认初始化设为 `'map'` 规避连接前回退渲染 |
| `src/amr_vision/amr_vision/fleet_manager_ai.py` | CORRIDOR_SEGMENTS/WAIT_POINTS 改井字主干道；publish_fleet_state 发布 corridor_segments + wait_points |
| `src/amr_vision/launch/amr_vision_fleet.launch.py` | 将各节点（包括 `fleet_manager_ai` / `tf_relay` / `nav2_goal_bridge` 等）的 `use_sim_time` 绑定到 `launch_gazebo` 变量以消除 `fleet-only` 模式下因缺少 `/clock` 产生的时钟死锁；给 `fleet_manager_ai` 节点补齐传入 `'world_file': world_file` 参数，解决后端因参数缺失导致 `world_name` 始终判定为 `'warehouse'` 引发的前端回退错位 |

> 均为 Python/JS/launch 改动，`--symlink-install` 下：前端浏览器刷新即生效；后端改动需重启 `fleet_manager_ai`（重启仿真）。

---

## 8. 验证记录（2026-06-20）

| 项 | 方式 | 结果 |
|----|------|------|
| 视频流 | curl `%2F` vs 字面 `/` 抓帧 | 0 帧 → 39 帧 ✅ |
| /map 元数据 | `ros2 topic echo /agv1/map --field info` | 998×598 @0.05, origin(-25,-15) ✅ |
| 走廊几何发布 | `ros2 topic echo /fleet/state` | corridor_segments 5 段 + wait_points 8 点 ✅ |
| 走廊布局合理性 | PIL 叠加走廊/zones 到 pgm 渲染 | 三纵避开货架、两横连通上下 ✅ |
| 相机流(重启后) | curl snapshot | HTTP 200 image/jpeg ✅ |
| 前端语法 | `node -c` + `bun build`（`~/.bun/bin/bun` v1.3.14） | 解析通过 ✅ |
| :8080 提供最新 | curl app.js grep | 含全部改动 ✅ |
| 非硬编码地图回退防错位 | 在大仓场景下关闭“真实地图”选项 | 地面、外墙和“幽灵货架”不再绘制，仅填充黑色背景底图，分区/AGV等数据驱动层尺度完美匹配 ✅ |

> 浏览器层的最终视觉确认由用户 Ctrl+Shift+R 刷新完成（CLI 已验证全部数据链路）。

---

## 9. 已知限制 / 后续

1. **走廊几何为人工标定**：当前井字坐标按本图货架/分区手工设定；换地图需相应更新 `fleet_manager_ai.CORRIDOR_SEGMENTS`/`WAIT_POINTS`（理想做法是从地图或参数自动生成）。
2. **VIEW 自适应假设地图居中**：非居中地图需引入中心偏移。
3. **YOLO 推理环境**：NumPy/opencv ABI 已修（`numpy<2` + `opencv-python<4.10`），详见 [web-camera-integration](web-camera-integration.md) §7。
4. **走廊路权对开阔大仓意义有限**：车少时几乎不触发；井字主要用于可视化与多车并发时的对撞避免。

---

## 10. 前端代码验证规范

为保证前端控制台与页面脚本在复杂大仓和多车并发调度中的健壮性，任何对 `app.js` 的修改，均需通过以下两步验证后再发布/提交：

1. **语法检查（Node.js）**：
   ```bash
   node -c src/amr_web/web/app.js
   ```
2. **打包验证（Bun）**：
   ```bash
   ~/.bun/bin/bun build src/amr_web/web/app.js --outdir /tmp/bun_build_test
   ```
   此打包验证命令可模拟前端发布过程，并自动捕获隐式 ES 模块引用、变量未定义或宏展开失败等深层语法与依赖问题。
