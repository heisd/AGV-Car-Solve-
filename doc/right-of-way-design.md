# 多 AGV 仓库路权系统设计文档

> 版本：v2.0 — 走廊段预约制路权  
> 更新日期：2026-06-19  
> 适用于：GazeboLib 仓库仿真项目（ROS2 Humble + Nav2 + Gazebo Classic）

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

当 AGV 因路权冲突需要让行时，不再"原地停车"（会堵路），而是驶向走廊入口处的
**指定等待点**——安全、不阻挡主通道。

### 4.1 等待点列表

| 等待点名称 | 坐标 (x, y) | 所属走廊 | 位置描述 |
|-----------|-------------|---------|----------|
| `corridor_east_south` | (5.5, -6.0) | 东走廊 | 东走廊南端入口 |
| `corridor_east_north` | (5.5, 6.0) | 东走廊 | 东走廊北端入口 |
| `corridor_west_south` | (-5.5, -6.0) | 西走廊 | 西走廊南端入口 |
| `corridor_west_north` | (-5.5, 6.0) | 西走廊 | 西走廊北端入口 |
| `corridor_north_east` | (6.0, 5.5) | 北走廊 | 北走廊东端入口 |
| `corridor_north_west` | (-6.0, 5.5) | 北走廊 | 北走廊西端入口 |
| `corridor_south_east` | (6.0, -5.5) | 南走廊 | 南走廊东端入口 |
| `corridor_south_west` | (-6.0, -5.5) | 南走廊 | 南走廊西端入口 |
| `corridor_center_south` | (0.0, -6.0) | 中央通道 | 中央通道南端入口 |
| `corridor_center_north` | (0.0, 6.0) | 中央通道 | 中央通道北端入口 |

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
规则 1（优先级裁决）：
    优先级高者获得段路权，低者在等待点让行

规则 2（同级裁决 — 防死锁）：
    优先级相同时，按 ns 字典序小者优先
    （如 agv1 < agv2 < agv3，确定且唯一，避免双方互让导致死锁）

规则 3（抢占规则）：
    高优先级车可抢占段路权，前提是原占有者尚未物理进入该段
    （已在段内的车不可被强制驱逐，安全第一）

规则 4（段离开检测）：
    AGV 离开段后自动释放预约 → 等待队列首车自动获得路权
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
       
3. 段预约与冲突处理
   for each AGV that is actively navigating:
       for each segment the AGV is currently in:
           if try_reserve_segment(ns, segment) succeeds:
               if AGV was yielding → clear yield, reissue_goal
           else:
               send AGV to nearest_wait_point
               add to _yielding set
               log conflict

4. 等待队列推进
   promote_waiting()  # 段空闲后，队首车获得路权

5. 清理不再需要让行的车
   for each AGV in _yielding:
       if no longer blocked → clear yield, reissue_goal
```

### 6.2 段预约算法 `try_reserve_segment()`

```
输入: ns (请求车), segment_name (目标段)
输出: True (获得路权) / False (需等待)

if 段无人占有 or 已被自己占有:
    设 _segment_owner[segment] = ns
    return True

owner = 段当前占有者
p_self = robot_priority(请求车)
p_owner = robot_priority(占有者)

if p_self > p_owner:
    if 占有者不在段内(尚未物理进入):
        抢占: _segment_owner[segment] = ns
        log "路权抢占"
        return True

将请求车加入 _segment_queue[segment] 等待队列
return False
```

### 6.3 等待队列推进 `promote_waiting()`

```
for each segment:
    if segment has no owner AND queue is not empty:
        promoted = queue.pop(0)  # 取队首
        _segment_owner[segment] = promoted
        if promoted in _yielding:
            _yielding.remove(promoted)
            reissue_goal(promoted)
        log "等待结束，获得路权"
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
| **确定性优先级** | 优先级不同时有明确胜负；相同时按 ns 字典序 → 不存在"对等互让" |
| **抢占规则** | 高优先级车可抢占未进入的段 → 打破等待链 |
| **等待点分散** | 让行车退到走廊入口 → 不在走廊中间堵路 |
| **段离开自动释放** | 车离开段后立即释放 → 不会长期占用 |
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

### 8.1 新增数据结构

```python
# 走廊段定义
CORRIDOR_SEGMENTS: dict[str, dict]
# 例: {'corridor_east': {'x_min': 4.5, 'x_max': 6.5, 'y_min': -6.5, 'y_max': 6.5}}

# 段拥有者映射
_segment_owner: dict[str, str]      # segment_name -> ns

# 段等待队列
_segment_queue: dict[str, list[str]]  # segment_name -> [ns1, ns2, ...]

# 等待点定义
WAIT_POINTS: dict[str, tuple[float, float]]  # wait_point_name -> (x, y)

# 当前让行等待目标
_wait_target: dict[str, tuple[float, float]]  # ns -> (x, y)
```

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
| **v2.0** | **2026-06-19** | **全面升级：走廊段预约制路权系统**（本文档） |

### v2.0 关键改进

- ✅ 从反应式 → 前瞻式（进入走廊前预约）
- ✅ 走廊感知（5 条命名走廊段 + 交叉区域）
- ✅ 等待点策略（10 个入口等待点，不堵路）
- ✅ 段预约/释放/队列推进（资源管理完整闭环）
- ✅ 抢占机制（高优先级车可抢占未进入的段）
- ✅ 死锁预防（确定性优先级 + 抢占 + 分散等待）
- ✅ Web 可视化（段占用 + 等待队列 + 段等待异常）
- ✅ 与 Nav2/区域互斥/碰撞检测三层协同
