#!/usr/bin/env python3
import os
import math
import yaml
import time
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import BatteryState

def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def stamp_now(node):
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    return msg

class AGVState:
    def __init__(self, ns):
        self.ns = ns
        self.pose = None  # (x,y)
        self.yaw = 0.0
        self.on_charger = False
        self.battery = 1.0  # 0..1
        self.state = 'IDLE'
        self.task = None  # {'id','pickup','dropoff','load_time','unload_time'}
        self.state_since = time.time()
        self.carrying = False
        self.nav_ready = False  # 该车 Nav2(导航栈) 是否已激活就绪
        self.stuck = False      # 朝目标长时间无进展（卡死/被堵）
        self.plan_poses = []    # 规划路径

class FleetManager(Node):
    """
    Minimal fleet / task manager:
      - Assigns pickup->dropoff tasks from a queue
      - Drives AGVs via /<ns>/goal_pose (used by your follower)
      - Considers battery thresholds and uses charger zones
      - Simulates loading/unloading by timed delays
      - Publishes /<ns>/carrying_load Bool for visualization/logic
    """
    def __init__(self):
        super().__init__('fleet_manager')

        # Params
        self.declare_parameter('robot_namespaces', ['agv1'])
        self.declare_parameter('tasks_file', '')
        self.declare_parameter('zones_yaml', '')
        self.declare_parameter('tasks_yaml', '')
        self.declare_parameter('battery_low_threshold', 0.20)
        self.declare_parameter('battery_resume_threshold', 0.60)
        self.declare_parameter('goal_reach_dist', 0.25)
        self.declare_parameter('charge_check_period', 1.0)
        self.declare_parameter('load_unload_poll_period', 0.2)
        self.declare_parameter('battery_topic_type', 'auto')  # 'auto'|'battery_state'|'float32'
        self.declare_parameter('charger_zone_names', ['charger_1','charger_2'])
        # 每车空闲待命点 [x1,y1,x2,y2,...] (与 robot_namespaces 同序)；空则用内置默认。
        self.declare_parameter('home_xy', [])
        self.declare_parameter('require_nav_ready', False)
        self.declare_parameter('world_file', '')

        self.ns_list = list(self.get_parameter('robot_namespaces').value)
        self.low_thr = float(self.get_parameter('battery_low_threshold').value)
        self.resume_thr = float(self.get_parameter('battery_resume_threshold').value)
        self.reach_dist = float(self.get_parameter('goal_reach_dist').value)
        self.batt_topic_type = str(self.get_parameter('battery_topic_type').value)
        self.charger_zone_names = list(self.get_parameter('charger_zone_names').value)
        self.require_nav_ready = bool(self.get_parameter('require_nav_ready').value)

        world_file = str(self.get_parameter('world_file').value or '').strip()
        self.world_name = os.path.splitext(os.path.basename(world_file))[0] if world_file else 'warehouse'

        # 空闲待命点：作业完成后回到各自独立的开阔走廊点，避免赖在作业区互相阻挡/相撞。
        # 默认点都在东/西走廊空地，彼此分散且远离取/卸货/充电区（坐标匹配 warehouse.world）。
        DEFAULT_HOME = {'agv1': (5.5, -1.5), 'agv2': (-5.5, 2.0), 'agv3': (5.5, 3.0)}
        home_flat = [float(v) for v in (self.get_parameter('home_xy').value or [])]
        self.home = {}
        for i, ns in enumerate(self.ns_list):
            if 2 * i + 1 < len(home_flat):
                self.home[ns] = (home_flat[2 * i], home_flat[2 * i + 1])
            elif ns in DEFAULT_HOME:
                self.home[ns] = DEFAULT_HOME[ns]

        # Load zones/tasks from YAML if provided, else from params
        tasks_file = str(self.get_parameter('tasks_file').value).strip()

        if tasks_file and os.path.exists(tasks_file):
            with open(tasks_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            self.zones = data.get('zones', {}) or {}
            self.task_queue = data.get('tasks', []) or []
        else:
            # Fallback: allow passing YAML as a string param (still not required by your launch)
            zones_yaml = str(self.get_parameter('zones_yaml').value or '').strip()
            tasks_yaml = str(self.get_parameter('tasks_yaml').value or '').strip()

            self.zones = yaml.safe_load(zones_yaml) if zones_yaml else {}
            if not isinstance(self.zones, dict):
                self.get_logger().warn("zones_yaml did not parse to a dict; ignoring.")
                self.zones = {}

            parsed_tasks = yaml.safe_load(tasks_yaml) if tasks_yaml else []
            if parsed_tasks is None:
                parsed_tasks = []
            if not isinstance(parsed_tasks, list):
                self.get_logger().warn("tasks_yaml did not parse to a list; ignoring.")
                parsed_tasks = []
            self.task_queue = parsed_tasks

        # Internal state per AGV
        self.agv = {ns: AGVState(ns) for ns in self.ns_list}

        # Publishers/subscribers per AGV
        self.goal_pub = {}
        self.carry_pub = {}
        for ns in self.ns_list:
            self.goal_pub[ns] = self.create_publisher(PoseStamped, f'/{ns}/goal_pose', 10)
            self.carry_pub[ns] = self.create_publisher(Bool, f'/{ns}/carrying_load', 10)
            self.create_subscription(Odometry, f'/{ns}/ground_truth', lambda msg, ns=ns: self.cb_odom(ns, msg), 10)
            self.create_subscription(Bool, f'/{ns}/on_charger', lambda msg, ns=ns: self.cb_on_charger(ns, msg), 10)
            self.create_subscription(Bool, f'/{ns}/nav_ready', lambda msg, ns=ns: self.cb_nav_ready(ns, msg), 10)
            self.create_subscription(Path, f'/{ns}/plan', lambda msg, ns=ns: self.cb_plan(ns, msg), 10)
            # Battery
            if self.batt_topic_type in ('auto','battery_state'):
                self.create_subscription(BatteryState, f'/{ns}/battery_state', lambda msg, ns=ns: self.cb_batt_state(ns, msg), 10)
            if self.batt_topic_type in ('auto','float32'):
                self.create_subscription(Float32, f'/{ns}/battery_percentage', lambda msg, ns=ns: self.cb_batt_float(ns, msg), 10)

        # Timers
        self.control_timer = self.create_timer(0.25, self.step)  # main loop

        # Goal re-send watchdog: nav2_goal_bridge buffers goals received during
        # Nav2 activation and ignores identical re-sends while a goal is active,
        # so we no longer blindly re-publish on a timer (that only spammed logs).
        # Instead we re-send a goal ONLY when the robot makes no progress toward
        # it for _goal_resend_interval — i.e. navigation stalled or aborted and
        # needs to be re-triggered.
        self._goal_last_sent: dict = {ns: 0.0 for ns in self.ns_list}
        self._goal_resend_interval = 12.0  # s with no progress before re-sending
        self._goal_progress_eps = 0.30     # m of closing distance counted as progress
        # per-AGV stall watchdog: {'best_d': closest distance so far, 't': time of last progress}
        self._goal_progress: dict = {ns: None for ns in self.ns_list}

        # 多车防撞 — 区域互斥预约：一个取/卸货/充电区同一时刻只允许一台车作为目标。
        self._zone_owner: dict = {}              # zone_name -> ns
        self._home_last_sent: dict = {ns: 0.0 for ns in self.ns_list}
        self._home_resend_interval = 12.0        # s，空闲回待命点的重发间隔（卡住会重试）
        self.standby_tol = 0.6                   # m，到待命点的容差

        # 充电优先让行：去充电的车(TO_CHARGER)在路径冲突时享有更高优先级，
        # 其它车进入其走廊段时需停车让行，待充电车通过后恢复。
        self._yielding: set = set()              # 当前正在让行的车 ns

        # ---- 走廊段路权系统（每拍重建占用 · 原子获取 · 同向共享） ----
        # 仓库走廊划分为命名段。每拍从零重建段占用：物理在段者锁定该段，新进入者
        # 按优先级原子获取其全部需求段（拿不全则一段不占、退到入口等待点让行）。
        # 同一段允许多台"同向"车跟车通行，仅对向/满载才互斥。详见 doc/right-of-way-design.md。
        self.CORRIDOR_SEGMENTS = {
            'corridor_east':      {'x_min': 4.5, 'x_max': 6.5, 'y_min': -6.5, 'y_max': 6.5},
            'corridor_west':      {'x_min': -6.5, 'x_max': -4.5, 'y_min': -6.5, 'y_max': 6.5},
            'corridor_north':     {'x_min': -6.5, 'x_max': 6.5, 'y_min': 4.5, 'y_max': 6.5},
            'corridor_south':     {'x_min': -6.5, 'x_max': 6.5, 'y_min': -6.5, 'y_max': -4.5},
            'corridor_center_ns': {'x_min': -1.0, 'x_max': 1.0, 'y_min': -6.5, 'y_max': 6.5},
        }
        # 段占用：segment -> [ns]（同向可多车，列表顺序=进入先后）。每拍重建。
        self._segment_occupants: dict = {s: [] for s in self.CORRIDOR_SEGMENTS}
        self._segment_dir: dict = {}            # segment -> 'positive'/'negative'（当前通行方向）
        self.SEGMENT_CAPACITY = 2               # 每段同向最多车数（>1 允许跟车；=1 退化为单行互斥）
        self.PLAN_LOOKAHEAD_M = 4.0             # 规划前瞻距离(m)：只预约前方这段路径穿过的走廊段
        # 等待点须落在"被让走廊之外"的相邻(垂直)走廊里：让行车停这里时不再是被让
        # 走廊的占用者，从而不会因"驶到入口即成为该段在位者"而自我解除让行、来回横跳。
        # 命名 corridor_<X>_<side> = 走廊 X 的 <side> 端口外侧(位于垂直走廊内)。
        self.WAIT_POINTS = {
            'corridor_east_south':    (4.0, -5.5),   # 南走廊内，东走廊南口外
            'corridor_east_north':    (4.0,  5.5),   # 北走廊内，东走廊北口外
            'corridor_west_south':    (-4.0, -5.5),  # 南走廊内，西走廊南口外
            'corridor_west_north':    (-4.0,  5.5),  # 北走廊内，西走廊北口外
            'corridor_north_east':    (5.5,  4.0),   # 东走廊内，北走廊东口外
            'corridor_north_west':    (-5.5, 4.0),   # 西走廊内，北走廊西口外
            'corridor_south_east':    (5.5, -4.0),   # 东走廊内，南走廊东口外
            'corridor_south_west':    (-5.5, -4.0),  # 西走廊内，南走廊西口外
            'corridor_center_south':  (2.0, -5.5),   # 南走廊内，中央通道南口外
            'corridor_center_north':  (2.0,  5.5),   # 北走廊内，中央通道北口外
        }
        self._wait_target: dict = {}            # ns -> (x,y) 当前让行等待点目标
        self._wait_progress: dict = {}          # ns -> 让行去等待点的失速看护 {'best_d','t'}
        self._yield_blocked_seg: dict = {}      # ns -> 因哪个段而让行（前端"等待队列"显示）
        self._yield_since: dict = {}            # ns -> 进入让行的时刻（迟滞 + 死锁兜底计时）
        self._yield_min_dwell = 0.5             # s，让行最短保持时长（防边界抖动 1 拍来回切）
        self._deadlock_timeout = 8.0            # s，让行久堵兜底：超时则非"赢家"车强制后撤

        # 碰撞检测：两车中心距 < collision_dist 视为碰撞/危险接近，
        # 边沿触发告警(WARN→Web异常日志)，并在 /fleet/state 发布当前碰撞对供前端显示。
        self.collision_dist = 0.55               # m（车体半径 0.30，<0.55 即重叠/危险）
        self._collisions: list = []              # [{'a','b','d'}]
        self._collision_pairs: set = set()       # 已告警的碰撞对（去抖）
        self.collision_count = 0                 # 累计碰撞事件数

        # ---- Web / dispatch-center interface (consumed by the web operator panel via rosbridge) ----
        #   OUT: /fleet/state   (std_msgs/String, JSON)  full fleet snapshot, published ~3 Hz
        #   IN:  /fleet/add_task(std_msgs/String, JSON)  {"pickup":<zone>,"dropoff":<zone>[,"id","load_time","unload_time"]}
        self.fleet_state_pub = self.create_publisher(String, '/fleet/state', 10)
        self.create_subscription(String, '/fleet/add_task', self.cb_add_task, 10)
        self._added_task_seq = 0
        self.create_timer(0.33, self.publish_fleet_state)

        self.get_logger().info(f"FleetManager started. AGVs: {self.ns_list}, tasks in queue: {len(self.task_queue)}")

    # ======== Callbacks ========

    def cb_odom(self, ns, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.agv[ns].pose = (p.x, p.y)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.agv[ns].yaw = math.atan2(siny_cosp, cosy_cosp)

    def cb_plan(self, ns, msg: Path):
        self.agv[ns].plan_poses = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def cb_on_charger(self, ns, msg: Bool):
        self.agv[ns].on_charger = bool(msg.data)

    def cb_nav_ready(self, ns, msg: Bool):
        self.agv[ns].nav_ready = bool(msg.data)

    def cb_batt_state(self, ns, msg: BatteryState):
        # percentage may be 0..100 or 0..1 depending on your battery_sim; normalize
        pct = msg.percentage
        if pct > 1.5:  # looks like 0..100
            pct = pct / 100.0
        pct = max(0.0, min(1.0, pct))
        self.agv[ns].battery = pct

    def cb_batt_float(self, ns, msg: Float32):
        pct = msg.data
        if pct > 1.5:
            pct = pct / 100.0
        pct = max(0.0, min(1.0, pct))
        self.agv[ns].battery = pct

    # ======== Main control loop ========

    def step(self):
        # Update each AGV state machine
        for ns in self.ns_list:
            self.run_agv(ns)

        # 走廊段预约制路权（碰撞预防；按任务优先级裁决通行权）
        self.apply_traffic_rules()

        # 碰撞检测（监控两车危险接近，告警 + 发布给前端）
        self.check_collisions()

        # Assign tasks to idle AGVs
        self.assign_tasks()

    # ======== 碰撞检测 ========

    def check_collisions(self):
        """检测两车中心距 < collision_dist 的碰撞/危险接近；边沿触发 WARN 告警。"""
        cols = []
        active = set()
        names = [ns for ns in self.ns_list if self.agv[ns].pose is not None]
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                d = dist(self.agv[a].pose, self.agv[b].pose)
                if d < self.collision_dist:
                    pair = tuple(sorted((a, b)))
                    cols.append({'a': pair[0], 'b': pair[1], 'd': round(d, 2)})
                    active.add(pair)
                    if pair not in self._collision_pairs:   # 新发生 → 告警一次
                        self.collision_count += 1
                        self.get_logger().error(
                            f"⚠ 碰撞检测：{pair[0]} 与 {pair[1]} 危险接近 (间距 {d:.2f} m < "
                            f"{self.collision_dist} m)！累计 {self.collision_count} 次。")
        self._collision_pairs = active
        self._collisions = cols

    # ======== 路径冲突·优先级让行（碰撞预防） ========

    # 状态优先级（数值大者优先通行，低者让行）：
    #   充电(去充电/充电中) > 载货送货 > 取货 > 空闲返航
    PRIORITY = {
        'TO_CHARGER': 5, 'CHARGING': 5,
        'TO_DROPOFF': 4, 'UNLOADING': 4,
        'TO_PICKUP': 3, 'LOADING': 3,
        'IDLE': 1,
    }

    def robot_priority(self, agv):
        return self.PRIORITY.get(agv.state, 1)

    def detect_segments(self, pose):
        """返回 pose (x,y) 所在的所有走廊段名列表。"""
        if pose is None:
            return []
        x, y = pose
        segs = []
        for name, rect in self.CORRIDOR_SEGMENTS.items():
            if rect['x_min'] <= x <= rect['x_max'] and rect['y_min'] <= y <= rect['y_max']:
                segs.append(name)
        return segs

    def detect_segments_from_plan(self, plan_poses, max_ahead_m=None):
        """返回规划路径"前瞻范围内"穿过的走廊段名列表。
        只看车前方约 PLAN_LOOKAHEAD_M 米的路径，避免一占就锁住整条走廊（吞吐优化）。"""
        if not plan_poses:
            return []
        limit = self.PLAN_LOOKAHEAD_M if max_ahead_m is None else max_ahead_m
        segs = set()
        acc = 0.0
        prev = plan_poses[0]
        for p in plan_poses:
            acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
            prev = p
            if limit and acc > limit:
                break
            for name, rect in self.CORRIDOR_SEGMENTS.items():
                if rect['x_min'] <= p[0] <= rect['x_max'] and rect['y_min'] <= p[1] <= rect['y_max']:
                    segs.add(name)
        return list(segs)

    def heading_direction(self, agv, segment_name):
        """判断 AGV 在走廊段内的行进方向：'positive'/'negative'/'unknown'。
        东/西走廊取 y 分量，南/北走廊取 x 分量，中心南北走廊取 y 分量。"""
        yaw = agv.yaw
        if segment_name in ('corridor_east', 'corridor_west', 'corridor_center_ns'):
            return 'positive' if math.sin(yaw) > 0.3 else ('negative' if math.sin(yaw) < -0.3 else 'unknown')
        elif segment_name in ('corridor_north', 'corridor_south'):
            return 'positive' if math.cos(yaw) > 0.3 else ('negative' if math.cos(yaw) < -0.3 else 'unknown')
        return 'unknown'

    def _intended_dir(self, ns, segment_name):
        """车在某走廊段的预期行进方向：优先用规划路径在段内的净位移判断，
        无规划则回退到航向角（复用 heading_direction）。返回 positive/negative/unknown。"""
        agv = self.agv[ns]
        rect = self.CORRIDOR_SEGMENTS[segment_name]
        axis = 1 if segment_name in ('corridor_east', 'corridor_west', 'corridor_center_ns') else 0
        pts = [p for p in getattr(agv, 'plan_poses', [])
               if rect['x_min'] <= p[0] <= rect['x_max'] and rect['y_min'] <= p[1] <= rect['y_max']]
        if len(pts) >= 2:
            delta = pts[-1][axis] - pts[0][axis]
            if delta > 0.5:
                return 'positive'
            if delta < -0.5:
                return 'negative'
        return self.heading_direction(agv, segment_name)

    def _can_admit(self, ns, segment_name, occ_map, dir_map):
        """在本拍正在重建的占用状态 (occ_map/dir_map) 下，ns 能否进入 segment_name。
        规则：空段可进；同向且未达容量可并入车队（跟车）；对向或满载则不可进。
        注意：物理在位者已先行放入 occ_map，故任何车都无法挤走"已在段内"的车（安全第一）。"""
        occ = occ_map[segment_name]
        if ns in occ:
            return True
        if not occ:
            return True
        d = self._intended_dir(ns, segment_name)
        if d != 'unknown' and dir_map.get(segment_name) == d and len(occ) < self.SEGMENT_CAPACITY:
            return True
        return False

    def release_segments(self, ns, segment_name=None):
        """释放 ns 的走廊段占用与让行状态。segment_name=None 释放全部
        （任务完成 / 充电 / 回到待命点时调用）。"""
        segs = [segment_name] if segment_name else list(self._segment_occupants)
        for s in segs:
            occ = self._segment_occupants.get(s, [])
            if ns in occ:
                occ.remove(ns)
            if not occ:
                self._segment_dir.pop(s, None)
        if segment_name is None and hasattr(self.agv[ns], 'plan_poses'):
            self.agv[ns].plan_poses = []
        self._clear_yield(ns)

    def _enter_yield(self, ns, blocked_seg):
        """让 ns 进入让行状态：记录被挡段，发往最近且未被占用的等待点。"""
        self._yielding.add(ns)
        self._yield_blocked_seg[ns] = blocked_seg
        self._yield_since[ns] = time.time()
        self._wait_progress[ns] = None
        wp = self.nearest_wait_point(ns, blocked_seg)
        if wp:
            self._wait_target[ns] = wp
            self.send_goal_xy(ns, wp)
            self.get_logger().warn(
                f"[{ns}] 走廊段 {blocked_seg} 冲突 → 让行至等待点 ({wp[0]:.1f},{wp[1]:.1f})。")
        else:
            self.get_logger().warn(f"[{ns}] 走廊段 {blocked_seg} 冲突，未找到合适等待点。")

    def _clear_yield(self, ns):
        """解除 ns 的让行状态（不主动下发目标，由调用方决定是否 reissue）。"""
        self._yielding.discard(ns)
        self._yield_blocked_seg.pop(ns, None)
        self._wait_target.pop(ns, None)
        self._wait_progress.pop(ns, None)
        self._yield_since.pop(ns, None)

    def _resend_wait_if_stalled(self, ns):
        """让行车若长时间到不了等待点（Nav2 中止/失速），重发等待点目标。"""
        wp = self._wait_target.get(ns)
        agv = self.agv[ns]
        if not wp or agv.pose is None:
            return
        d = dist(agv.pose, wp)
        if d <= self.standby_tol:
            return
        now = time.time()
        prog = self._wait_progress.get(ns)
        if prog is None:
            self._wait_progress[ns] = {'best_d': d, 't': now}
            return
        if d < prog['best_d'] - self._goal_progress_eps:
            prog['best_d'] = d
            prog['t'] = now
            return
        if now - prog['t'] >= self._goal_resend_interval:
            self.send_goal_xy(ns, wp)
            prog['t'] = now
            self.get_logger().warn(
                f"[{ns}] 让行去等待点 {self._goal_resend_interval:.0f}s 无进展 → 重发。")

    def _force_retreat(self, ns):
        """死锁/久堵兜底：让 ns 彻底退出当前走廊段（释放占用 + 后撤到自身段入口），
        给"赢家"车让出整条通路，打破对角路口的相持。"""
        for s, occ in self._segment_occupants.items():
            if ns in occ:
                occ.remove(ns)
                if not occ:
                    self._segment_dir.pop(s, None)
        wp = self._retreat_wait_point(ns)
        if wp:
            self._wait_target[ns] = wp
            self.send_goal_xy(ns, wp)
        self._yield_since[ns] = time.time()      # 重置计时，给后撤动作留时间
        self.get_logger().warn(
            f"[{ns}] 路权久堵({self._deadlock_timeout:.0f}s) → 强制后撤让路。")

    def _retreat_wait_point(self, ns):
        """选 ns 当前所在段的等待点中、离"被挡段"最远的那个（即向后撤离方向）。"""
        agv = self.agv[ns]
        if agv.pose is None:
            return None
        my_segs = self.detect_segments(agv.pose)
        cands = [xy for seg in my_segs for name, xy in self.WAIT_POINTS.items()
                 if name.startswith(seg)]
        if not cands:
            cands = list(self.WAIT_POINTS.values())
        blocked = self._yield_blocked_seg.get(ns)
        if blocked and blocked in self.CORRIDOR_SEGMENTS:
            r = self.CORRIDOR_SEGMENTS[blocked]
            bc = ((r['x_min'] + r['x_max']) / 2.0, (r['y_min'] + r['y_max']) / 2.0)
            return max(cands, key=lambda xy: dist(xy, bc))
        return min(cands, key=lambda xy: dist(agv.pose, xy))

    def _segment_waiters(self):
        """{段名: [因该段而让行的车]} —— 供前端"等待队列"列显示。"""
        out = {}
        for ns, seg in self._yield_blocked_seg.items():
            out.setdefault(seg, []).append(ns)
        return out

    def nearest_wait_point(self, ns, segment_name):
        """为 ns 在指定走廊段找到最近的等待点，让车停在段入口外等候。
        会避开需要跨越当前占用车辆的阻挡等待点。"""
        agv = self.agv[ns]
        if agv.pose is None:
            return None
        # 收集该走廊段对应的等待点
        prefix = segment_name  # e.g. 'corridor_east'
        candidates = [(wp_name, wp_xy) for wp_name, wp_xy in self.WAIT_POINTS.items()
                      if wp_name.startswith(prefix)]
        if not candidates:
            # fallback：所有等待点中找最近的
            candidates = list(self.WAIT_POINTS.items())

        others = [o for o in self._segment_occupants.get(segment_name, []) if o != ns]
        owner_pose = self.agv[others[0]].pose if (others and self.agv[others[0]].pose) else None

        valid_candidates = []
        if owner_pose:
            is_ns = segment_name in ('corridor_east', 'corridor_west', 'corridor_center_ns')
            for wp_name, wp_xy in candidates:
                if is_ns:
                    # 南北走廊：owner_pose 的 y 坐标介于机器人和等待点之间则说明被阻挡
                    y_robot = agv.pose[1]
                    y_owner = owner_pose[1]
                    y_wp = wp_xy[1]
                    if (y_robot < y_owner < y_wp) or (y_wp < y_owner < y_robot):
                        continue
                else:
                    # 东西走廊：owner_pose 的 x 坐标介于机器人和等待点之间则说明被阻挡
                    x_robot = agv.pose[0]
                    x_owner = owner_pose[0]
                    x_wp = wp_xy[0]
                    if (x_robot < x_owner < x_wp) or (x_wp < x_owner < x_robot):
                        continue
                valid_candidates.append((wp_name, wp_xy))
        else:
            valid_candidates = candidates

        if not valid_candidates:
            valid_candidates = candidates

        # 等待点互斥：剔除已被其他让行车占用/锁定的等待点，避免两车挤同一点相撞。
        claimed = [t for n2, t in self._wait_target.items() if n2 != ns]
        best_wp = None
        best_d = float('inf')
        for wp_name, wp_xy in valid_candidates:
            if any(dist(wp_xy, c) <= self.standby_tol for c in claimed):
                continue
            d = dist(agv.pose, wp_xy)
            if d < best_d:
                best_d = d
                best_wp = wp_xy
        if best_wp is None:        # 候选都被占 → 退回不排除占用的最近点
            for wp_name, wp_xy in valid_candidates:
                d = dist(agv.pose, wp_xy)
                if d < best_d:
                    best_d = d
                    best_wp = wp_xy
        return best_wp

    def apply_traffic_rules(self):
        """走廊段路权（防撞核心）。每拍从零重建段占用，保证四条不变量：
          1) 原子获取——一台车要么拿到它需要的全部段，要么一段都不新占（杜绝
             "拿到一段、为另一段让行"导致的同 tick 决策互相覆盖与持有-等待死锁）；
          2) 禁止持有-等待——让行车不新占任何段，破坏死锁必要条件；
          3) 物理在位者绝对优先——已物理处于某段的车在本拍锁定该段，任何车（即便
             更高优先级）都不能挤走它（安全第一，避免段内对撞）；
          4) 隐式优先级抢占——按 (优先级降序, ns 升序) 处理新进入者：高优先级车先选段，
             低优先级车遇占用则在入口等待点让行，无需显式驱逐已入段的车。
        同向共享：同一段允许 SEGMENT_CAPACITY 台同向车跟车通行，仅对向/满载才互斥。
        详见 doc/right-of-way-design.md 与 doc/path-conflict-resolution.xml。"""
        # 0) 收集每台车的物理所在段 phys 与需求段 needed(=物理 + 前瞻规划)
        phys, needed, active = {}, {}, []
        for ns in self.ns_list:
            agv = self.agv[ns]
            if agv.pose is None:
                continue                       # 无位姿 → 不参与，本拍自动释放其占用
            home = self.home.get(ns)
            at_home = (agv.state == 'IDLE' and home is not None
                       and dist(agv.pose, home) <= self.standby_tol)
            if agv.state == 'CHARGING' or at_home:
                continue                       # 充电中/已到待命点 → 不占用走廊
            p = set(self.detect_segments(agv.pose))
            phys[ns] = p
            needed[ns] = p | set(self.detect_segments_from_plan(getattr(agv, 'plan_poses', [])))
            active.append(ns)

        # 1) 先把"物理在位者"放入新占用表（锁定，不可被剥夺）
        new_occ = {s: [] for s in self.CORRIDOR_SEGMENTS}
        new_dir = {}
        for ns in active:
            for s in phys[ns]:
                if not new_occ[s]:
                    new_dir[s] = self._intended_dir(ns, s)
                new_occ[s].append(ns)

        # 2) 新进入者按 (优先级降序, ns 升序) 原子获取需求段；拿不全则一段不占 → 让行
        order = sorted(active, key=lambda n: (-self.robot_priority(self.agv[n]), n))
        block_of = {}
        for ns in order:
            want_new = [s for s in needed[ns] if ns not in new_occ[s]]
            block = next((s for s in want_new
                          if not self._can_admit(ns, s, new_occ, new_dir)), None)
            if block is None:
                for s in want_new:             # 原子提交：全部进入
                    if not new_occ[s]:
                        new_dir[s] = self._intended_dir(ns, s)
                    new_occ[s].append(ns)
            block_of[ns] = block               # block 非空 = 禁止持有-等待，一段不新占

        # 3) 提交占用状态
        self._segment_occupants = new_occ
        self._segment_dir = new_dir

        # 4) 久堵兜底：让行过久者后撤；全局"赢家"(优先级最高且 ns 最小) 坚守不退
        yielders = [n for n in active if block_of[n] is not None]
        winner = min(yielders, key=lambda n: (-self.robot_priority(self.agv[n]), n)) if yielders else None

        # 5) 驱动让行/恢复（仅在状态切换时改目标，避免刷 goal）
        now = time.time()
        for ns in active:
            block = block_of[ns]
            if block is not None:
                if ns not in self._yielding:
                    self._enter_yield(ns, block)
                elif ns != winner and now - self._yield_since.get(ns, now) >= self._deadlock_timeout:
                    self._force_retreat(ns)
                else:
                    self._yield_blocked_seg[ns] = block      # 更新被挡段（用于前端显示）
                    self._resend_wait_if_stalled(ns)
            elif ns in self._yielding:
                # 迟滞：让行最短保持 _yield_min_dwell，防边界抖动 1 拍来回切
                if now - self._yield_since.get(ns, 0.0) >= self._yield_min_dwell:
                    self._clear_yield(ns)
                    self.reissue_goal(ns)

        # 6) 已退出 active（充电/到家/失位）但仍标记让行的车 → 清理
        for ns in list(self._yielding):
            if ns not in active:
                self._clear_yield(ns)

    def reissue_goal(self, ns):
        """让行结束后，按当前状态重新下发目标。"""
        agv = self.agv[ns]
        if agv.state == 'TO_CHARGER':
            tz = self.target_zone_of(ns)
            if tz:
                self.nav_to_zone(ns, tz)
        elif agv.state == 'TO_PICKUP' and agv.task:
            self.nav_to_zone(ns, agv.task['pickup'])
        elif agv.state == 'TO_DROPOFF' and agv.task:
            self.nav_to_zone(ns, agv.task['dropoff'])
        elif agv.state == 'IDLE':
            self._home_last_sent[ns] = 0.0        # 下一拍重发回待命点

    # ======== Assignment ========

    def assign_tasks(self):
        if not self.task_queue:
            return
        # candidates: idle AGVs with battery >= low_thr
        idle = [a for a in self.agv.values() if a.state == 'IDLE' and a.pose is not None
                and a.battery >= self.low_thr and a.ns not in self._yielding]
        if self.require_nav_ready:
            idle = [a for a in idle if a.nav_ready]
        if not idle:
            return

        # Greedy: pick best (AGV, task) by distance to pickup
        for agv in idle:
            # pick best task for this AGV
            best = None
            best_d = float('inf')
            for idx, t in enumerate(self.task_queue):
                pz = self.zones.get(t['pickup'])
                if not pz:
                    continue
                # 区域互斥：取/卸货区被别的车预约时跳过，避免两车涌向同一区域相撞
                if not self.zone_free(t['pickup'], agv.ns) or not self.zone_free(t.get('dropoff'), agv.ns):
                    continue
                d = dist(agv.pose, (pz['cx'], pz['cy']))
                if d < best_d:
                    best_d = d
                    best = (idx, t)
            if best is None:
                continue
            # Assign（同时预约取货区与卸货区，整段任务内独占）
            idx, task = best
            agv.task = task
            agv.state = 'TO_PICKUP'
            agv.state_since = time.time()
            self.reserve(task['pickup'], agv.ns)
            self.reserve(task.get('dropoff'), agv.ns)
            self._home_last_sent[agv.ns] = 0.0
            self.nav_to_zone(agv.ns, task['pickup'])
            self.get_logger().info(
                f"[{agv.ns}] Assigned task {task.get('id','?')} → pickup {task['pickup']} "
                f"(预约 {task['pickup']} + {task.get('dropoff')})")
            # Remove from queue
            self.task_queue.pop(idx)
            # Assign one task at a time per loop to keep it simple
            break

    # ======== Per-AGV FSM ========

    def run_agv(self, ns):
        agv = self.agv[ns]
        now = time.time()
        # If battery low and not charging nor going to charger, send to the nearest charger
        if agv.state in ('IDLE','TO_PICKUP','TO_DROPOFF') and agv.battery < self.low_thr and not agv.on_charger:
            cz_name = self.nearest_charger(agv.pose)
            if cz_name:
                self.release_all(ns)          # 放弃当前任务区预约
                self.reserve(cz_name, ns)     # 预约充电区
                agv.state = 'TO_CHARGER'
                agv.state_since = now
                self.nav_to_zone(ns, cz_name)
                self.get_logger().info(f"[{ns}] Battery low ({agv.battery:.2f}). Heading to charger {cz_name}.")
            return

        # If charging and recovered
        if agv.state in ('CHARGING','TO_CHARGER') and (agv.battery >= self.resume_thr) and agv.on_charger:
            self.release_all(ns)              # 释放充电区
            agv.state = 'IDLE'
            agv.state_since = now
            self.get_logger().info(f"[{ns}] Battery recovered ({agv.battery:.2f}). Back to IDLE.")
            return

        # State logic
        if agv.state == 'IDLE':
            # Publish carrying false just to be explicit
            self.set_carrying(ns, False)
            # 空闲且不在待命点 → 回各自独立的待命点，清空作业区、分散车辆（防撞核心）。
            # 定时重发：若回程导航被中止/卡住，每 _home_resend_interval 重试一次。
            home = self.home.get(ns)
            if (ns not in self._yielding and home and agv.pose is not None
                    and dist(agv.pose, home) > self.standby_tol):
                if now - self._home_last_sent.get(ns, 0.0) >= self._home_resend_interval:
                    self.send_goal_xy(ns, home)
                    self._home_last_sent[ns] = now
                    self.get_logger().info(
                        f"[{ns}] Idle — 返回待命点 ({home[0]:.1f}, {home[1]:.1f}).")
            return

        if agv.state == 'TO_CHARGER':
            if self.at_zone(ns, agv.pose, self.target_zone_of(ns)):
                agv.state = 'CHARGING'
                agv.state_since = now
                self.get_logger().info(f"[{ns}] Arrived at charger. Waiting to charge...")
            else:
                self._resend_goal_if_due(ns, self.target_zone_of(ns), now)
            return

        if agv.state == 'CHARGING':
            # Passive: wait until resume_thr while on_charger
            return

        if agv.state == 'TO_PICKUP':
            if agv.pose is None:
                return
            tz = self.target_zone_of(ns)
            if self.at_zone(ns, agv.pose, tz):
                agv.state = 'LOADING'
                agv.state_since = now
                self.get_logger().info(f"[{ns}] At pickup {agv.task['pickup']}. Loading...")
            else:
                self._resend_goal_if_due(ns, tz, now)
            return

        if agv.state == 'LOADING':
            load_time = float(agv.task.get('load_time', 3.0))
            if now - agv.state_since >= load_time:
                self.set_carrying(ns, True)
                self.release(agv.task['pickup'], ns)   # 离开取货区，释放给其他车
                agv.state = 'TO_DROPOFF'
                agv.state_since = now
                self.nav_to_zone(ns, agv.task['dropoff'])
                self.get_logger().info(f"[{ns}] Loaded. Heading to dropoff {agv.task['dropoff']}.")
            return

        if agv.state == 'TO_DROPOFF':
            if agv.pose is None:
                return
            tz = self.target_zone_of(ns)
            if self.at_zone(ns, agv.pose, tz):
                agv.state = 'UNLOADING'
                agv.state_since = now
                self.get_logger().info(f"[{ns}] At dropoff {agv.task['dropoff']}. Unloading...")
            else:
                self._resend_goal_if_due(ns, tz, now)
            return

        if agv.state == 'UNLOADING':
            unload_time = float(agv.task.get('unload_time', 3.0))
            if now - agv.state_since >= unload_time:
                self.set_carrying(ns, False)
                self.get_logger().info(f"[{ns}] Task {agv.task.get('id','?')} complete.")
                self.release_all(ns)          # 释放该车占用的所有区域
                agv.task = None
                agv.state = 'IDLE'
                agv.state_since = now
                self._home_last_sent[ns] = 0.0  # 触发下一拍立即返回待命点
            return

    # ======== Helpers ========

    def goal_xy_of(self, zone_name):
        """Navigation target for a zone: the reachable approach point gx,gy if
        defined, else the semantic center cx,cy. Zones whose center sits inside a
        physical obstacle (pickup pallet, charger cabinet) carry gx,gy so Nav2's
        global planner gets a goal in free space instead of an unreachable cell."""
        z = self.zones[zone_name]
        return float(z.get('gx', z['cx'])), float(z.get('gy', z['cy']))

    # ---- 区域互斥预约（多车防撞） ----
    def zone_free(self, zone, ns):
        """zone 是否可被 ns 占用：无人预约或已被自己预约。"""
        if not zone:
            return True
        owner = self._zone_owner.get(zone)
        return owner is None or owner == ns

    def reserve(self, zone, ns):
        if zone:
            self._zone_owner[zone] = ns

    def release(self, zone, ns):
        if zone and self._zone_owner.get(zone) == ns:
            del self._zone_owner[zone]

    def release_all(self, ns):
        for z in [z for z, o in list(self._zone_owner.items()) if o == ns]:
            del self._zone_owner[z]
        self.release_segments(ns)  # 同时释放走廊段预约

    def send_goal_xy(self, ns, xy):
        """发布一个非区域的原始目标点（用于返回待命点）。"""
        msg = stamp_now(self)
        msg.pose.position.x = float(xy[0])
        msg.pose.position.y = float(xy[1])
        msg.pose.orientation.w = 1.0
        self.goal_pub[ns].publish(msg)
        setattr(self, f'_target_zone_{ns}', None)
        self._goal_last_sent[ns] = time.time()
        self._goal_progress[ns] = None

    def nav_to_zone(self, ns, zone_name):
        gx, gy = self.goal_xy_of(zone_name)
        msg = stamp_now(self)
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.orientation.w = 1.0
        self.goal_pub[ns].publish(msg)
        # Track current navigation target per AGV for zone checks
        setattr(self, f'_target_zone_{ns}', zone_name)
        self._goal_last_sent[ns] = time.time()
        # Reset the stall watchdog: re-baseline progress on the next step().
        self._goal_progress[ns] = None

    def _resend_goal_if_due(self, ns, zone_name, now):
        """Re-send the goal only if the robot has stalled — no progress toward
        the goal for _goal_resend_interval. Catches a navigation goal that was
        aborted (e.g. transient BT failure) without spamming re-sends while the
        robot is travelling normally (nav2_goal_bridge ignores those anyway)."""
        agv = self.agv[ns]
        if not zone_name or agv.pose is None:
            return
        if ns in self._yielding:               # 让行中：保持停车，不重发
            return
        gx, gy = self.goal_xy_of(zone_name)
        d = dist(agv.pose, (gx, gy))
        prog = self._goal_progress.get(ns)
        if prog is None:                       # first observation after a fresh send
            self._goal_progress[ns] = {'best_d': d, 't': now}
            return
        if d < prog['best_d'] - self._goal_progress_eps:
            prog['best_d'] = d                 # closing in — refresh watchdog
            prog['t'] = now
            agv.stuck = False                  # 有进展 → 清卡死标志
            return
        if now - prog['t'] >= self._goal_resend_interval:
            agv.stuck = True                   # 长时间无进展 → 标记卡死（前端异常显示）
            self.get_logger().warn(
                f"[{ns}] No progress toward {zone_name} for "
                f"{self._goal_resend_interval:.0f}s (d={d:.2f} m) — re-sending goal.")
            self.nav_to_zone(ns, zone_name)    # this resets _goal_progress[ns]

    def target_zone_of(self, ns):
        return getattr(self, f'_target_zone_{ns}', None)

    def at_zone(self, ns, pose_xy, zone_name):
        if not pose_xy or not zone_name:
            return False
        z = self.zones.get(zone_name)
        if not z:
            return False
        # Primary: reached the actual navigation goal (approach point). With Nav2
        # the robot stops near gx,gy (an obstacle-free cell), which may lie OUTSIDE
        # the semantic cx,cy rectangle, so arrival is judged against the goal point.
        gx, gy = self.goal_xy_of(zone_name)
        if dist(pose_xy, (gx, gy)) <= max(self.reach_dist, 0.5):
            return True
        # Secondary: physically inside the semantic zone rectangle (free-center zones).
        x, y = pose_xy
        hx = float(z['sx']) / 2.0 + 0.05
        hy = float(z['sy']) / 2.0 + 0.05
        return (abs(x - float(z['cx'])) <= hx) and (abs(y - float(z['cy'])) <= hy)

    def nearest_charger(self, pose_xy):
        if not pose_xy:
            return None
        best = None
        best_d = float('inf')
        for name in self.charger_zone_names:
            z = self.zones.get(name)
            if not z:
                continue
            d = dist(pose_xy, (z['cx'], z['cy']))
            if d < best_d:
                best_d = d
                best = name
        return best

    def set_carrying(self, ns, val: bool):
        self.carry_pub[ns].publish(Bool(data=val))
        self.agv[ns].carrying = val

    # ======== Web / dispatch-center interface ========

    def cb_add_task(self, msg: String):
        """Append a transport task submitted from the web panel."""
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f"/fleet/add_task: invalid JSON ({e})")
            return
        pickup = data.get('pickup')
        dropoff = data.get('dropoff')
        if pickup not in self.zones or dropoff not in self.zones:
            self.get_logger().warn(
                f"/fleet/add_task: unknown zone(s) pickup={pickup} dropoff={dropoff}; "
                f"known={list(self.zones.keys())}")
            return
        self._added_task_seq += 1
        task = {
            'id': str(data.get('id') or f'W{self._added_task_seq}'),
            'pickup': pickup,
            'dropoff': dropoff,
            'load_time': float(data.get('load_time', 3.0)),
            'unload_time': float(data.get('unload_time', 3.0)),
        }
        self.task_queue.append(task)
        self.get_logger().info(
            f"[dispatch] queued task {task['id']}: {pickup} -> {dropoff} (queue={len(self.task_queue)})")

    def publish_fleet_state(self):
        """Publish a JSON snapshot of the whole fleet for the web operator panel."""
        agvs = []
        for ns in self.ns_list:
            a = self.agv[ns]
            home = self.home.get(ns)
            agvs.append({
                'ns': ns,
                'x': round(a.pose[0], 3) if a.pose else None,
                'y': round(a.pose[1], 3) if a.pose else None,
                'yaw': round(a.yaw, 3),
                'battery': round(a.battery, 3),
                'state': a.state,
                'task': (a.task.get('id') if a.task else None),
                'carrying': bool(a.carrying),
                'on_charger': bool(a.on_charger),
                'nav_ready': bool(a.nav_ready),    # Nav2 导航栈是否就绪（未就绪 Web 会标红）
                'stuck': bool(a.stuck),            # 朝目标长时间无进展
                'home_x': round(home[0], 3) if home else None,
                'home_y': round(home[1], 3) if home else None,
            })
        zones = {
            name: {'cx': float(z['cx']), 'cy': float(z['cy']),
                   'sx': float(z.get('sx', 1.0)), 'sy': float(z.get('sy', 1.0)),
                   'gx': float(z.get('gx', z['cx'])), 'gy': float(z.get('gy', z['cy']))}
            for name, z in self.zones.items()
        }

        # 任务分配总览（供前端"任务分配情况"面板）：已分配的（在某台车上）+ 排队中的（未分配）
        tasks = []
        for ns in self.ns_list:
            a = self.agv[ns]
            if a.task:
                tasks.append({
                    'id': str(a.task.get('id', '?')),
                    'pickup': a.task.get('pickup'),
                    'dropoff': a.task.get('dropoff'),
                    'status': 'assigned',      # 已分配/执行中
                    'agv': ns,
                    'agv_state': a.state,
                })
        for t in self.task_queue:
            tasks.append({
                'id': str(t.get('id', '?')),
                'pickup': t.get('pickup'),
                'dropoff': t.get('dropoff'),
                'status': 'queued',            # 排队中，等待空闲车
                'agv': None,
                'agv_state': None,
            })
        idle_agvs = [ns for ns in self.ns_list if self.agv[ns].state == 'IDLE']

        # ---- 系统异常总览（供前端"系统异常"面板统一显示） ----
        # level: error(红) / warn(橙) / info(蓝)
        anomalies = []
        for c in self._collisions:               # 碰撞/危险接近（最严重）
            anomalies.append({'level': 'error', 'ns': f"{c['a']}↔{c['b']}",
                              'type': '碰撞', 'msg': f"危险接近 {c['d']} m"})
        for ns in self.ns_list:
            a = self.agv[ns]
            if a.pose is None:
                anomalies.append({'level': 'error', 'ns': ns, 'type': '离线',
                                  'msg': '无位姿/未上报（Gazebo 未生成或里程计缺失）'})
                continue
            if self.require_nav_ready and not a.nav_ready:
                anomalies.append({'level': 'warn', 'ns': ns, 'type': '导航未就绪',
                                  'msg': 'Nav2 导航栈未激活'})
            if a.battery < self.low_thr and not a.on_charger and a.state != 'CHARGING':
                anomalies.append({'level': 'warn', 'ns': ns, 'type': '低电量',
                                  'msg': f"电量 {a.battery*100:.0f}% < {self.low_thr*100:.0f}%"})
            if a.stuck and a.state in ('TO_PICKUP', 'TO_DROPOFF', 'TO_CHARGER'):
                anomalies.append({'level': 'warn', 'ns': ns, 'type': '导航卡死',
                                  'msg': '朝目标长时间无进展（被堵/规划失败）'})
            if ns in self._yielding:
                wp = self._wait_target.get(ns)
                if wp:
                    anomalies.append({'level': 'info', 'ns': ns, 'type': '段等待',
                                      'msg': f'路权等待中，停在等待点 ({wp[0]:.1f},{wp[1]:.1f})'})
                else:
                    anomalies.append({'level': 'info', 'ns': ns, 'type': '段等待',
                                      'msg': '路权等待中'})

        payload = {
            'stamp': time.time(),
            'world_name': self.world_name,
            'agvs': agvs,
            'queued_tasks': len(self.task_queue),
            'zones': zones,
            'charger_zones': self.charger_zone_names,
            'zone_owner': dict(self._zone_owner),   # zone_name -> ns（区域预约，防撞可视化）
            # 走廊段占用（同向可多车）：segment_owner 取首位占用者(兼容旧前端=字符串)，
            # segment_occupants 给全量占用，segment_dir 给方向，segment_queue 给"正等待该段"的让行车。
            'segment_owner': {s: lst[0] for s, lst in self._segment_occupants.items() if lst},
            'segment_occupants': {s: list(lst) for s, lst in self._segment_occupants.items() if lst},
            'segment_dir': dict(self._segment_dir),
            'segment_queue': self._segment_waiters(),   # 走廊段等待车（因该段而让行）
            'tasks': tasks,                         # 任务分配总览
            'idle_agvs': idle_agvs,                 # 空闲车列表（"是否有空闲小车"）
            'yielding': sorted(self._yielding),     # 路权等待中的车（走廊段冲突）
            'collisions': self._collisions,         # 当前碰撞/危险接近对 [{a,b,d}]
            'collision_count': self.collision_count,  # 累计碰撞事件数
            'anomalies': anomalies,                 # 系统异常总览
            'require_nav_ready': self.require_nav_ready, # 是否要求导航就绪
        }
        self.fleet_state_pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()