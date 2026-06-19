# 多 AGV 路权逻辑优化方案

> 版本：v3.0 — 走廊段占用制重构
> 日期：2026-06-20
> 适用：GazeboLib 仓库仿真（ROS2 Humble + Nav2 + Gazebo Classic）
> 实现：`src/amr_description/amr_description/fleet_manager.py` · `apply_traffic_rules()` 及其辅助方法
> 配套：`doc/right-of-way-design.md`（设计） · `doc/path-conflict-resolution.xml`（结构化规格）

本文记录对 v2.0「走廊段预约制路权」的代码审查结论、本次（v3.0）落地的优化、验证结果与残余风险。

---

## 1. 审查发现的问题（v2.0）

代码审查针对 `fleet_manager.py` 的 `apply_traffic_rules` / `try_reserve_segment` /
`release_segments` / `promote_waiting` / `nearest_wait_point` 等链路，发现以下缺陷：

| 编号 | 严重度 | 位置 | 问题 |
|------|--------|------|------|
| **D1** | 🔴 严重 | `current_segs = list(set(...))` + 逐段 `try_reserve` + 整车单一 `_yielding` | **多段获取非原子**：`set` 迭代顺序不定，循环里后处理的段会覆盖前段的让行决定。过路口（需两段）时，对某段预约成功会**撤销**对另一被占段的让行 → 约一半概率径直驶入被占段对头。行为不可复现。 |
| **D2** | 🔴 严重 | `try_reserve_segment` 抢占分支 | **抢占不检查占有者是否已物理在段内**，且被抢者未被立即制动、仍朝原目标行驶 → 可能逼停/穿插正在窄道行驶的车，制造对向相撞。与设计文档自述的"抢占需占有者未进入"规则相悖。 |
| **D3** | 🔴 严重 | 让行车不释放已占段（仅"物理驶离"才释放） | **持有-等待**：路口双段竞争且两车同优先级（最常见，如都在送货）时形成循环等待 → 死锁。 |
| **D4** | 🟠 中 | `heading_direction` 定义后零调用 | **死代码**：方向判断未接线 → 同向车也被单行互斥，无法跟车，吞吐被腰斩。 |
| **D5** | 🟠 中 | `detect_segments_from_plan` 预约整条规划路径 | 每条走廊是贯穿整侧的单一段，**远处即锁死整侧** → 3 车被严重串行化，死锁面变大。 |
| **D6** | 🟠 中 | 等待点 / 队列 / 状态字段 | 等待点**无失速看护**（Nav2 中止则卡死）、**不互斥**（两车挤同点相撞）；队列 `insert(0)` 抢占 + `pop(0)` 提升导致**优先级反转**；`_yielding`/`_wait_target` 易失步；段边界硬阈值**无迟滞**（真实定位噪声会刷爆 `/goal_pose`）。 |

---

## 2. 优化方案（已落地 v3.0）

核心思想：**放弃"跨拍持久的段预约 + 等待队列"，改为每个控制拍从零重建段占用**。
这一处架构改变同时根除 D1/D2/D3，并让 D4/D5/D6 的修复变得自然。

### 方案 A（安全/活性 · 必做）— 原子化 + 禁止持有-等待 + 安全抢占

每拍 `apply_traffic_rules()` 重建占用表 `_segment_occupants`：

1. **物理在位者锁定**（不变量 I3）：先把"物理已在段内"的车放进占用表，任何车都不能挤走 → 安全第一，绝不把车逼入段内对撞（修 **D2**）。
2. **按序处理 + 原子获取**（I1/I4）：新进入者按 `(优先级降序, ns 升序)` 处理；一台车要么拿到它需要的**全部**段，要么**一段都不新占**。消除 `set` 顺序不确定与同 tick 决策互相覆盖（修 **D1**）。
3. **禁止持有-等待**（I2）：让行车不持有任何段 → 破坏死锁必要条件（修 **D3**）。
4. **隐式优先级抢占**：高优先级车被先处理，自然先得段；低优先级随后遇占用即让行。**无需**显式驱逐"已入段"的车 —— 安全地实现了 D2 想要却没做对的抢占语义。

> 同优先级冲突由 `(优先级, ns)` 全序确定性裁决，不存在对等互让。

### 方案 B（吞吐）— 同向共享 + 前瞻限距

- **B1 同向共享**（I5，修 **D4**）：接上原死代码 `heading_direction`，新增 `_intended_dir()`（优先用规划净位移定向，回退航向角）。同一段允许 `SEGMENT_CAPACITY=2` 台**同向**车跟车通行，仅**对向/满载**才互斥。车距由 Nav2 costmap 维持。
- **B2 前瞻限距**（修 **D5**）：`detect_segments_from_plan` 按累计弧长只看前方 `PLAN_LOOKAHEAD_M=4.0m`，不再一占锁死整侧走廊。

### 方案 C（鲁棒性 · 修 D6）

- **等待点失速看护** `_resend_wait_if_stalled`：去等待点长时间无进展则重发（去掉旧版对让行车的无条件跳过）。
- **等待点互斥**：`nearest_wait_point` 剔除已被其他让行车锁定（≤`standby_tol`）的等待点。
- **让行状态原子化**：`_enter_yield` / `_clear_yield` 统一增删 `_yielding`/`_wait_target`/`_yield_blocked_seg`/`_wait_progress`/`_yield_since`，不再失步。
- **迟滞** `_yield_min_dwell=0.5s`：让行最短保持时长，防边界抖动 1 拍来回切。

### 死锁兜底（针对残余风险）

`_force_retreat` + `_retreat_wait_point`：出现真实相持（≥2 车互堵）且某让行车久堵超 `_deadlock_timeout=8.0s` 时，
非"赢家"（优先级最高且 ns 最小）的车释放占用并后撤到自身段入口，打破对角路口相持，保证活性。

---

## 3. 代码变更摘要

| 动作 | 符号 |
|------|------|
| 删除 | `try_reserve_segment`、`promote_waiting`、`_segment_owner`、`_segment_queue` |
| 重写 | `apply_traffic_rules`（每拍重建）、`release_segments`（占用制）、`nearest_wait_point`（占用+互斥）、`detect_segments_from_plan`（前瞻限距） |
| 新增 | `_intended_dir`、`_can_admit`、`_enter_yield`、`_clear_yield`、`_resend_wait_if_stalled`、`_force_retreat`、`_retreat_wait_point`、`_segment_waiters` |
| 接线 | `heading_direction`（原死代码，现被 `_intended_dir` 调用） |
| 新状态 | `_segment_occupants`、`_segment_dir`、`SEGMENT_CAPACITY`、`PLAN_LOOKAHEAD_M`、`_wait_progress`、`_yield_blocked_seg`、`_yield_since`、`_yield_min_dwell`、`_deadlock_timeout` |
| Web 载荷 | `segment_owner`(=首位占用者,兼容旧前端) + 新增 `segment_occupants`/`segment_dir`；`segment_queue`=因段让行的车（类型仍为 list，前端无需改动） |

---

## 4. 验证

通过 `rclpy` 实例化 `FleetManager`（3 车），用合成位姿/规划驱动 `apply_traffic_rules()`，断言核心不变量（脚本：临时 `test_traffic.py`，21/21 通过）：

| 场景 | 断言 | 结果 |
|------|------|------|
| S1 对头冲突（对向） | 恰一台进入 + 胜者确定（多次重复不变）+ 另一台让行 | ✅ |
| S2 同向跟车 | 两车共享同段、无人让行 | ✅ |
| S3 容量溢出 | 同向第三台让行（段占用上限 2） | ✅ |
| S4 路口（多段）冲突 | 被挡车**保留**物理段、**不**获取被占段（无持有-等待） | ✅ |
| S5 真实双车互堵 | 两车均让行 → 久堵后**输家强制后撤**、**赢家坚守** | ✅ |
| S6 载荷形状 | `segment_owner` 为字符串、`segment_queue` 为列表（前端兼容） | ✅ |

`python3 -m py_compile fleet_manager.py` 通过。

**整机回归（2 车，headless）已执行**：真机 Nav2 栈下全程 `collision_count=0`，让行→等待点→恢复闭环正确，
按 `sim-run-stop-protocol` 启停、无僵尸进程。受 WSL2 上 Nav2/AMCL(`map→odom`) 拉起偶发失败影响（与本路权改动无关，
launch 已注明拉满 3 车不可靠、默认 2 车），未能每次凑齐"两车同时在线 + 受控对头"窗口。

> 复现两车对头：两车均 `nav_ready` 后，经 `/fleet/add_task` 注入**不相交、路径对穿**的任务对
> `pickup_A→dropoff_B` 与 `pickup_B→dropoff_A`，观察 `/fleet/state` 的 `yielding` / `segment_occupants` / `collision_count`。
> （`A↔B` 这类同区任务会被区域互斥正确地串行到一台车，无法构成对头。）

---

## 5. 残余风险与未来工作

- **等待点越界（2 车整机回归发现，已修）**：原 10 个等待点落在被让走廊矩形**内**，让行车驶到即成为该段物理在位者 → 按 I3 自动解除让行 → 再次尝试进入 → 再让行，**来回横跳**不前进。已将等待点移到被让走廊**之外**的相邻(垂直)走廊里（`fleet_manager.py:WAIT_POINTS`，详见 `right-of-way-design.md` §4）。
- **整侧段粒度的对角路口相持**（低风险）：走廊为"贯穿整侧"的单一段，两车分别物理占据相邻两段并互需对方时会短暂相持。目前由"久堵后撤兜底"保证活性（最坏 `_deadlock_timeout` 秒破解）。
  - **未来 B2+：走廊细分子段**（如 `east_s/east_mid/east_n`）+ 增量预约，把锁粒度从"整侧"降到"段"，从根上消除该相持并进一步提吞吐。
- **方向判定依赖规划/航向**：定位噪声大时 `_intended_dir` 可能判 `unknown`（此时保守按互斥处理，安全但偏保守）。
- **参数标定**：`SEGMENT_CAPACITY` / `PLAN_LOOKAHEAD_M` / `_deadlock_timeout` 当前为经验值，建议在三车满载场景下实测微调。
- **上真实定位前**：建议把段边界判定加入更强迟滞（进入用原 rect、离开要求连续 N 拍在外），当前 `_yield_min_dwell` 仅作最小防抖。
