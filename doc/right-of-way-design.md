# 多 AGV 仓库路权系统设计文档

> 版本：v3.0 — 走廊段占用制路权（每拍重建·原子获取·同向共享）  
> 更新日期：2026-06-20  
> 适用于：GazeboLib 仓库仿真项目（ROS2 Humble + Nav2 + Gazebo Classic）
>
> 算法已于 v3.0 重构；最新结构化规格见 [`path-conflict-resolution.xml`](path-conflict-resolution.xml)，审查与优化记录见 [`path-conflict-optimization-plan.md`](path-conflict-optimization-plan.md)。

---

## 1. 背景与问题

### 1.1 项目概述

本项目在 Gazebo 仿真环境中运行 2~3 台 AGV（自动引导车）执行仓库取货/卸货/充电任务。
仓库内部 14×14m，四周为单行走廊（宽约 2m），中间为货架区。AGV 通过 Nav2 导航栈进行
路径规划与动态避障。

### 1.2 旧系统问题（v1.0 — 反应式优先级让行）

| 问题 | 描述 |
|------|------|
| **纯反应式** | 仅在两车距离 < 1.6m 时触发让行，对窄走廊来说为时已晚 |
| **无走廊感知** | 不知道 AGV 在哪条走廊上，无法预判即将发生的对头冲突 |
| **无死锁预防** | 两车在同一走廊对向行驶时可能死锁（互相挡路） |
| **原地停车堵路** | 让行时原地停车反而堵住高优先级车的必经之路 |
| **无前瞻性** | 不考虑未来路径，只看当前位置 |

### 1.3 行业调研

通过调研学术文献和工业实践，我们总结了以下主流多 AGV 路权管理方案：

#### 方案对比

| 方案 | 原理 | 优点 | 缺点 | 适用场景 |
|------|------|------|------|----------|
| **区域互斥控制** | 将工作区划分为区域，限制每区域同时进入的 AGV 数量 | 实现简单、可靠 | 区域粒度大则效率低 | 小规模车队 |
| **时间窗预约 (TWR)** | 在时间-空间维度预约路径段，确保不同 AGV 不同时占用同一格 | 高吞吐量、前瞻性强 | 需精确时间估计、计算复杂 | 大规模高密度 |
| **走廊段预约制** | 将走廊划分为命名段，每段同一时刻只允许一台车 | 中等复杂度、走廊感知、死锁可控 | 段粒度需合理设计 | 中小规模、窄走廊 |
| **人工势场 (APF)** | 障碍物产生斥力、目标产生引力 | 实时反应 | 局部最优、振荡 | 开阔区域 |
| **冲突搜索 (CBS)** | 高层搜索路径集合，低层解决碰撞 | 全局最优 | 计算量大 | 离线规划 |
| **Petri 网建模** | 形式化建模验证无死锁 | 数学保证 | 建模复杂 | 学术验证 |

#### 选型决策

> **选定方案：走廊段预约制 + 优先级裁决 + 等待点策略**

理由：
1. 本仓库走廊窄（~2m），天然适合将走廊建模为互斥资源
2. 2~3 台车的规模不需要复杂的 CBS 或 TWR
3. 走廊段预约与 Nav2 路径规划自然融合——调度层控制"是否放行"，Nav2 负责"怎么走"
4. 等待点策略解决了旧方案"原地停车堵路"的问题

---

## 2. 系统架构

### 2.1 总体架构

```
┌─────────────────────────────────────────────────────────┐
│                    fleet_manager (调度中心)                │
│                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────┐  │
│  │   任务分配    │  │  路权管理器   │  │   碰撞检测    │  │
│  │ assign_tasks  │  │ (核心变更)   │  │check_collisions│  │
│  └──────────────┘  └──────────────┘  └───────────────┘  │
│                           │                              │
│         ┌─────────────────┼─────────────────┐           │
│         │                 │                 │           │
│  ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐    │
│  │ 段占用检测   │  │  段预约/释放  │  │ 等待点管理   │    │
│  │detect_segments│ │reserve/release│ │ wait_points  │    │
│  └─────────────┘  └─────────────┘  └─────────────┘    │
│                                                         │
│  ┌──────────────┐  ┌──────────────┐                     │
│  │  区域互斥    │  │  电量/充电    │                     │
│  │ zone_owner   │  │  管理(保留)   │                     │
│  └──────────────┘  └──────────────┘                     │
└─────────────────────────────────────────────────────────┘
          │                    │
   /fleet/state (JSON)   /<ns>/goal_pose
          │                    │
    ┌─────▼─────┐      ┌──────▼──────┐
    │  Web 面板  │      │nav2_goal_bridge│
    └───────────┘      └─────────────┘
```

### 2.2 路权管理器在控制循环中的位置

```python
def step(self):                    # 250ms 主循环
    for ns in self.ns_list:
        self.run_agv(ns)           # 1. 各车状态机推进
    self.apply_traffic_rules()     # 2. 路权管理（核心！）
    self.check_collisions()        # 3. 碰撞检测（安全兜底）
    self.assign_tasks()            # 4. 任务分配
```

---

## 3. 走廊段定义

### 3.1 仓库布局与走廊划分

```
          -6.5       -4.5  -1  0  1   4.5       6.5
           │          │    │     │    │          │
    6.5 ───┼──────────┼────┼──┬──┼────┼──────────┼──── 6.5
           │  NORTH   │    │  │  │    │  NORTH   │
    4.5 ───┼──────────┼────┼──┼──┼────┼──────────┼──── 4.5
           │          │    │  │  │    │          │
           │   WEST   │ 货架区│C│ 货架区│   EAST   │
           │  走廊    │    │N │  │    │   走廊   │
           │          │    │S │  │    │          │
           │          │    │  │  │    │          │
   -4.5 ───┼──────────┼────┼──┼──┼────┼──────────┼──── -4.5
           │  SOUTH   │    │  │  │    │  SOUTH   │
   -6.5 ───┼──────────┼────┼──┴──┼────┼──────────┼──── -6.5
           │          │    │     │    │          │
          -6.5       -4.5  -1  0  1   4.5       6.5

    EAST  = corridor_east      (x: 4.5~6.5,  y: -6.5~6.5)
    WEST  = corridor_west      (x: -6.5~-4.5, y: -6.5~6.5)
    NORTH = corridor_north     (x: -6.5~6.5,  y: 4.5~6.5)
    SOUTH = corridor_south     (x: -6.5~6.5,  y: -6.5~-4.5)
    CNS   = corridor_center_ns (x: -1.0~1.0,  y: -6.5~6.5)
```

### 3.2 走廊段属性

| 段名称 | X 范围 | Y 范围 | 方向 | 宽度 | 用途 |
|--------|--------|--------|------|------|------|
| `corridor_east` | [4.5, 6.5] | [-6.5, 6.5] | 南北 | 2.0m | 东侧走廊，连接 pickup_A ↔ dropoff_A |
| `corridor_west` | [-6.5, -4.5] | [-6.5, 6.5] | 南北 | 2.0m | 西侧走廊，连接 pickup_B ↔ charger_1 |
| `corridor_north` | [-6.5, 6.5] | [4.5, 6.5] | 东西 | 2.0m | 北横向走廊，连接 pickup_A ↔ pickup_B |
| `corridor_south` | [-6.5, 6.5] | [-6.5, -4.5] | 东西 | 2.0m | 南横向走廊，连接 dropoff_A ↔ charger_1 |
| `corridor_center_ns` | [-1.0, 1.0] | [-6.5, 6.5] | 南北 | 2.0m | 中央南北通道（货架间过道） |

### 3.3 交叉区域

走廊段存在交叉（intersection），一台 AGV 可能同时处于两个段中：

| 交叉位置 | 段 1 | 段 2 | 坐标范围 |
|----------|------|------|----------|
| 东北角 | corridor_east | corridor_north | x∈[4.5,6.5], y∈[4.5,6.5] |
| 东南角 | corridor_east | corridor_south | x∈[4.5,6.5], y∈[-6.5,-4.5] |
| 西北角 | corridor_west | corridor_north | x∈[-6.5,-4.5], y∈[4.5,6.5] |
| 西南角 | corridor_west | corridor_south | x∈[-6.5,-4.5], y∈[-6.5,-4.5] |
| 北中央 | corridor_center_ns | corridor_north | x∈[-1.0,1.0], y∈[4.5,6.5] |
| 南中央 | corridor_center_ns | corridor_south | x∈[-1.0,1.0], y∈[-6.5,-4.5] |

> 交叉区域规则：进入交叉区域的 AGV 需要同时持有两个段的预约。

---

## 4. 等待点定义

当 AGV 因路权冲突需要让行时，不再"原地停车"（会堵路），而是驶向**指定等待点**。

> **关键约束（v3.0 修正）**：等待点必须落在**被让走廊之外**的相邻(垂直)走廊里。
> 否则让行车一驶到入口就成为被让走廊的物理在位者 → 按 I3 自动解除让行 → 再次尝试进入 →
> 再让行，**来回横跳**且不前进（2 车整机回归实测到此现象）。因此每个 `corridor_<X>_<side>`
> 等待点放在走廊 X 的 <side> 端口**外侧**（位于与之垂直的走廊内）。

### 4.1 等待点列表

| 等待点名称 | 坐标 (x, y) | 所属走廊 | 位置描述 |
|-----------|-------------|---------|----------|
| `corridor_east_south` | (4.0, -5.5) | 东走廊 | 东走廊南口外（位于南走廊内） |
| `corridor_east_north` | (4.0, 5.5) | 东走廊 | 东走廊北口外（位于北走廊内） |
| `corridor_west_south` | (-4.0, -5.5) | 西走廊 | 西走廊南口外（位于南走廊内） |
| `corridor_west_north` | (-4.0, 5.5) | 西走廊 | 西走廊北口外（位于北走廊内） |
| `corridor_north_east` | (5.5, 4.0) | 北走廊 | 北走廊东口外（位于东走廊内） |
| `corridor_north_west` | (-5.5, 4.0) | 北走廊 | 北走廊西口外（位于西走廊内） |
| `corridor_south_east` | (5.5, -4.0) | 南走廊 | 南走廊东口外（位于东走廊内） |
| `corridor_south_west` | (-5.5, -4.0) | 南走廊 | 南走廊西口外（位于西走廊内） |
| `corridor_center_south` | (2.0, -5.5) | 中央通道 | 中央通道南口外（位于南走廊内） |
| `corridor_center_north` | (2.0, 5.5) | 中央通道 | 中央通道北口外（位于北走廊内） |

### 4.2 等待点选择策略

让行 AGV 选择离自己最近的、属于被占段的等待点：

```
nearest_wait_point(ns, segment_name):
    过滤出 segment_name 对应的所有等待点
    返回距离 AGV 当前位置最近的那个
```

---

## 5. 优先级规则

### 5.1 状态优先级（数值大者优先通行）

| 状态 | 优先级值 | 说明 |
|------|---------|------|
| `TO_CHARGER` / `CHARGING` | 5 | 充电车最高优先级（缺电不能被堵） |
| `TO_DROPOFF` / `UNLOADING` | 4 | 载货送货（已携带货物，尽快完成） |
| `TO_PICKUP` / `LOADING` | 3 | 去取货（尚未携带货物） |
| `IDLE` | 1 | 空闲/返回待命点（最低优先级） |

### 5.2 冲突裁决规则

```
规则 1（物理在位锁定）：
    物理已在段内的车本拍无条件锁定该段，任何车（即便更高优先级）都不能挤走它（安全第一）

规则 2（优先级 + 同级裁决）：
    新进入者按 (优先级降序, ns 升序) 处理；同级按 ns 字典序（agv1 < agv2 < agv3）
    确定且唯一，不存在对等互让

规则 3（原子获取 + 隐式抢占）：
    一台车要么拿到需要的全部段、要么一段都不新占；高优先级车被先处理即先得段，
    低优先级随后遇占用即让行——无需显式驱逐已入段的车

规则 4（同向共享）：
    同一段允许 SEGMENT_CAPACITY 台同向车跟车通行，仅对向/满载才互斥

规则 5（久堵后撤）：
    出现真实相持(≥2 车互堵)且让行超 _deadlock_timeout 时，非"赢家"车释放占用并后撤，打破相持
```

---

## 6. 核心算法流程

### 6.1 主循环 `apply_traffic_rules()`

```
每 250ms 执行一次:

1. 段占用检测
   for each AGV with valid pose:
       current_segments = detect_segments(agv.pose)
       
2. 自动释放已离开的段
   for each segment owned by this AGV:
       if AGV not physically in this segment anymore:
           release_segment(ns, segment_name)
       
3. 重建占用（物理在位者锁定）
   new_occ = {seg: []}
   for each active AGV:
       for seg in detect_segments(agv.pose):   # 物理在段者本拍锁定该段
           new_occ[seg].append(ns)

4. 新进入者按 (优先级降序, ns 升序) 原子获取
   for ns in sorted(active, key=(-priority, ns)):
       want_new = needed[ns] 中尚未占有的段
       if 每个 want_new 段都 _can_admit:        # 原子：全有或全无
           全部加入 new_occ（提交）
       else:
           标记让行（一段都不新占 → 禁止持有-等待）

5. 清理不再需要让行的车
   for each AGV in _yielding:
       if no longer blocked → clear yield, reissue_goal
```

### 6.2 可进入判定 `_can_admit()`

```
输入: ns, segment_name, 本拍正在重建的 occ_map/dir_map
输出: True (可进入) / False (需让行)

occ = occ_map[segment]
if ns in occ:            return True   # 已在该段
if occ 为空:             return True   # 空段可进
d = _intended_dir(ns, segment)         # 规划净位移定向，回退航向角
if d 已知 and dir_map[segment] == d and len(occ) < SEGMENT_CAPACITY:
    return True                        # 同向且未满 → 并入车队(跟车)
return False                           # 对向 / 满载 → 让行

注：物理在位者已先行放入 occ_map，故任何车都无法挤走"已在段内"的车。
```

### 6.3 让行驱动与久堵后撤（替代旧的队列推进）

```
每拍重建占用后（无持久队列），按状态切换驱动目标：

winner = 让行车中 (优先级最高, ns 最小) 者
for ns in active:
    if 本拍被挡:
        if 之前未让行:     _enter_yield(ns)   # 去最近且互斥的等待点
        elif ns != winner and 让行已超 _deadlock_timeout:
                           _force_retreat(ns) # 释放占用 + 后撤，打破相持
        else:              _resend_wait_if_stalled(ns)  # 等待点失速看护
    elif 之前在让行 and 让行已超 _yield_min_dwell(迟滞):
                           _clear_yield(ns); reissue_goal(ns)  # 恢复原目标
```

---

## 7. 死锁预防

### 7.1 可能的死锁场景

```
场景：循环等待
  AGV_A 持有 corridor_east，等待 corridor_north
  AGV_B 持有 corridor_north，等待 corridor_east
  → 两车互相等待，永远无法前进
```

### 7.2 预防机制

| 机制 | 说明 |
|------|------|
| **禁止持有-等待** | 让行车不持任何段 → 破坏死锁必要条件（核心保证） |
| **每拍重建 + 确定性顺序** | 按 (优先级, ns) 全序处理 → 无对等互让/抖动选主，无陈旧预约 |
| **物理在位者锁定** | 不驱逐已在段内的车 → 不会因抢占把车逼入对撞 |
| **久堵后撤兜底** | 残余的"对角路口双物理在位互堵"由非赢家强制后撤打破 → 保证活性 |
| **任务分配过滤** | 让行中的车不参与新任务分配 → 不会引入更多冲突 |

### 7.3 与区域互斥的协同

原有的区域互斥（`_zone_owner`）继续生效，保护 pickup/dropoff/charger 这些端点区域。
走廊段预约保护的是**路径上的走廊资源**。两套系统协同工作：

```
区域互斥：保护"目的地"（取货点、卸货点、充电站）
走廊段预约：保护"路径"（走廊通道）
碰撞检测：安全兜底（最后防线，告警 + 报前端）
```

---

## 8. 数据结构

### 8.1 核心数据结构（v3.0）

```python
# 走廊段定义
CORRIDOR_SEGMENTS: dict[str, dict]
# 例: {'corridor_east': {'x_min': 4.5, 'x_max': 6.5, 'y_min': -6.5, 'y_max': 6.5}}

# 段占用（同向可多车，列表顺序=进入先后）；每拍重建
_segment_occupants: dict[str, list[str]]   # segment_name -> [ns, ...]
_segment_dir: dict[str, str]               # segment_name -> 'positive'/'negative'
SEGMENT_CAPACITY: int = 2                   # 每段同向最多车数
PLAN_LOOKAHEAD_M: float = 4.0               # 规划前瞻限距(m)

# 等待点
WAIT_POINTS: dict[str, tuple[float, float]] # name -> (x, y)
_wait_target: dict[str, tuple]              # ns -> 当前让行/后撤等待点
_wait_progress: dict[str, dict]             # ns -> 去等待点失速看护
_yield_blocked_seg: dict[str, str]          # ns -> 因哪个段而让行
_yield_since: dict[str, float]              # ns -> 进入让行时刻（迟滞 + 久堵计时）
```

> 已移除 v2.0 的 `_segment_owner`（单拥有者）与 `_segment_queue`（等待队列）——每拍重建占用后不再需要持久预约/队列。

### 8.2 保留的数据结构

```python
# 区域互斥（保留，保护端点区域）
_zone_owner: dict[str, str]         # zone_name -> ns

# 让行车集合（保留，语义不变）
_yielding: set[str]                 # 正在让行的车 ns

# 碰撞检测（保留，安全兜底）
_collisions: list[dict]
_collision_pairs: set
collision_count: int
```

---

## 9. Web 面板集成

### 9.1 `/fleet/state` 新增字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `segment_owner` | `dict` | `{段名: AGV ns}` — 当前走廊段占用情况 |
| `segment_queue` | `dict` | `{段名: [ns1, ns2]}` — 各段等待队列（仅非空段） |

### 9.2 异常面板新增类型

| level | type | 说明 |
|-------|------|------|
| `info` | `段等待` | AGV 因路权冲突在等待点停车，显示等待点坐标 |

---

## 10. 与 Nav2 的协同

### 10.1 分层设计

```
调度层（fleet_manager）：决定"是否放行" — 路权管理
导航层（Nav2）：决定"怎么走" — 路径规划 + 动态避障
运动层（DWB controller）：决定"如何控制" — 速度/角速度
```

路权系统工作在调度层，通过 `/<ns>/goal_pose` 控制 AGV 的导航目标：
- **放行**：下发实际任务目标（取货点/卸货点/充电站）
- **让行**：下发等待点目标（走廊入口安全位置）
- **恢复**：让行结束后重新下发原任务目标

### 10.2 Nav2 不感知路权

Nav2 只负责规划从当前位置到目标的路径。路权系统通过改变目标点来间接控制 AGV 行为：
- Nav2 的 costmap 仍然提供实时避障（安全保障层）
- 路权系统在更高层面避免走廊冲突（效率保障层）

---

## 11. 参考资料

1. **Zone Control for AGV Systems** — UPC Barcelona (Vis et al.)
   - 区域互斥控制的经典文献
2. **Improved Dynamic Resource Reservation (IDRR)** — ResearchGate
   - 动态资源预约方法，支持多段同时预约
3. **Conflict-Based Search (CBS)** — NIH/PMC
   - 多 Agent 路径规划的冲突搜索算法
4. **Petri Net Modeling for Deadlock-Free AGV Scheduling** — arXiv
   - Petri 网建模验证无死锁调度
5. **Open-RMF (Robotics Middleware Framework)** — GitHub
   - 开源多机器人车队管理中间件

---

## 12. 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| v1.0 | 2026-06 | 初始版本：反应式优先级让行 + 原地停车 |
| v1.1 | 2026-06 | 修复：改为横向让到走廊内侧（`yield_pullaside_target`） |
| v2.0 | 2026-06-19 | 走廊段预约制路权系统（存在 D1~D6 实现缺陷，详见优化方案） |
| **v3.0** | **2026-06-20** | **重构：走廊段占用制（每拍重建 + 原子获取 + 禁止持有-等待 + 物理在位锁定 + 隐式抢占 + 同向共享 + 久堵后撤），修复 D1~D6** |

### v2.0 关键改进

- ✅ 从反应式 → 前瞻式（进入走廊前预约）
- ✅ 走廊感知（5 条命名走廊段 + 交叉区域）
- ✅ 等待点策略（10 个入口等待点，不堵路）
- ✅ 段预约/释放/队列推进（资源管理完整闭环）
- ✅ 抢占机制（高优先级车可抢占未进入的段）
- ✅ 死锁预防（确定性优先级 + 抢占 + 分散等待）
- ✅ Web 可视化（段占用 + 等待队列 + 段等待异常）
- ✅ 与 Nav2/区域互斥/碰撞检测三层协同

### v3.0 关键改进（修复 v2.0 的 D1~D6）

- ✅ **每拍重建占用** → 杜绝陈旧预约与 set 顺序不确定（修 D1）
- ✅ **原子获取 + 禁止持有-等待** → 根除路口循环等待死锁（修 D3）
- ✅ **物理在位者锁定 + 隐式优先级抢占** → 安全抢占，绝不把车逼入段内对撞（修 D2）
- ✅ **同向共享（接上 heading_direction）** → 同向跟车，吞吐回升（修 D4）
- ✅ **规划前瞻限距 4m** → 不再一占锁死整侧走廊（修 D5）
- ✅ **等待点失速看护 + 互斥 + 让行迟滞 + 状态原子化**（修 D6）
- ✅ **久堵后撤兜底** → 残余对角路口相持的活性保证
- ✅ rclpy 逻辑测试 21/21 通过（对头/同向/容量/路口无持有-等待/死锁后撤/载荷兼容）
