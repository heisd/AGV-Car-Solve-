#!/usr/bin/env python3
"""
fleet_manager_ai.py

Semantically-aware fleet/task manager for the amr_vision stack.

This is the full FleetManager (FSM + corridor right-of-way + zone reservation +
collision detection + /fleet/state web telemetry + /fleet/add_task ingestion),
with an added YOLO semantic-perception layer driven by camera_detection_node:

  - person  → cancel Nav2 goal, stop in place, wait for "clear"
  - pallet  → do NOT cancel goal (Nav2 reroutes via costmap), publish reduced
              cmd_vel so the robot slows while Nav2 plans around it
  - clear   → if previously stopped for a person, re-send last goal and resume

Self-contained copy of amr_description/fleet_manager.py (the amr_vision package
deliberately duplicates its sibling modules) — kept behaviourally identical so
the web operator panel (amr_web) works against this manager exactly as it does
against the geometric one. Semantic hooks are folded into nav_to_zone,
_resend_goal_if_due, reissue_goal and step().

NOTE on coordinates: CORRIDOR_SEGMENTS / WAIT_POINTS below are tuned for the
±7 m warehouse world (matching amr_web/app.js). The amr_vision tasks.yaml zones
live on the larger map.world (±20 m), so on that map the corridor right-of-way
stays inert (robots never enter these segments) and the panel's right-of-way
table shows all-idle. Retune these rectangles to the active map to make
geometric right-of-way fire there.
"""

import os
import math
import yaml
import time
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import BatteryState


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


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
        self.manual_charge = False  # 操作员手动派去充电（充满前不提前返回）
        self.plan_poses = []    # 规划路径

        # --- Semantic perception state (YOLO layer) ---
        self.semantic_obstacle = 'clear'      # clear|person|pallet|unknown_obstacle
        self.semantic_stop_active = False     # True while stopped for a person
        self.last_nav_goal = None             # last goal sent; used to resume after a stop


class FleetManagerAI(Node):
    """
    Fleet / task manager with corridor right-of-way + YOLO semantic layer:
      - Assigns pickup->dropoff tasks from a queue
      - Drives AGVs via /<ns>/goal_pose
      - Battery thresholds + charger zones
      - Simulated loading/unloading by timed delays
      - Corridor-segment right-of-way (collision prevention)
      - Collision monitoring + /fleet/state telemetry + /fleet/add_task ingestion
      - Semantic perception: stop for person, slow for pallet, resume on clear
    """
    def __init__(self):
        super().__init__('fleet_manager_ai')

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
        self.declare_parameter('charger_zone_names', ['charger_1', 'charger_2'])
        # 每车空闲待命点 [x1,y1,x2,y2,...] (与 robot_namespaces 同序)；空则用内置默认。
        self.declare_parameter('home_xy', [])
        self.declare_parameter('require_nav_ready', False)
        self.declare_parameter('world_file', '')
        # 卡死车默认让出走廊路权（释放占用 + 强制后撤），避免异常占用走廊堵死他车。
        self.declare_parameter('release_corridor_on_stuck', True)
        # 手动充电充满阈值：手动派去充电的车充到该比例才回 IDLE（自动充电仍用 resume_thr）。
        self.declare_parameter('full_charge_threshold', 0.99)

        self.ns_list = list(self.get_parameter('robot_namespaces').value)
        self.low_thr = float(self.get_parameter('battery_low_threshold').value)
        self.resume_thr = float(self.get_parameter('battery_resume_threshold').value)
        self.reach_dist = float(self.get_parameter('goal_reach_dist').value)
        self.batt_topic_type = str(self.get_parameter('battery_topic_type').value)
        self.charger_zone_names = list(self.get_parameter('charger_zone_names').value)
        self.require_nav_ready = bool(self.get_parameter('require_nav_ready').value)
        self.release_corridor_on_stuck = bool(self.get_parameter('release_corridor_on_stuck').value)
        self.full_charge_thr = float(self.get_parameter('full_charge_threshold').value)

        world_file = str(self.get_parameter('world_file').value or '').strip()
        self.world_name = os.path.splitext(os.path.basename(world_file))[0] if world_file else 'warehouse'

        # 空闲待命点：作业完成后回到各自独立的开阔走廊点，避免赖在作业区互相阻挡/相撞。
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
        self.cmd_vel_pub = {}   # used to publish reduced speed on pallet detection
        for ns in self.ns_list:
            self.goal_pub[ns] = self.create_publisher(PoseStamped, f'/{ns}/goal_pose', 10)
            self.carry_pub[ns] = self.create_publisher(Bool, f'/{ns}/carrying_load', 10)
            self.cmd_vel_pub[ns] = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
            self.create_subscription(Odometry, f'/{ns}/ground_truth', lambda msg, ns=ns: self.cb_odom(ns, msg), 10)
            self.create_subscription(Bool, f'/{ns}/on_charger', lambda msg, ns=ns: self.cb_on_charger(ns, msg), 10)
            self.create_subscription(Bool, f'/{ns}/nav_ready', lambda msg, ns=ns: self.cb_nav_ready(ns, msg), 10)
            self.create_subscription(Path, f'/{ns}/plan', lambda msg, ns=ns: self.cb_plan(ns, msg), 10)
            # Battery
            if self.batt_topic_type in ('auto', 'battery_state'):
                self.create_subscription(BatteryState, f'/{ns}/battery_state', lambda msg, ns=ns: self.cb_batt_state(ns, msg), 10)
            if self.batt_topic_type in ('auto', 'float32'):
                self.create_subscription(Float32, f'/{ns}/battery_percentage', lambda msg, ns=ns: self.cb_batt_float(ns, msg), 10)
            # Semantic perception (YOLO layer)
            self.create_subscription(String, f'/{ns}/camera/obstacle_semantic', lambda msg, ns=ns: self._semantic_callback(ns, msg), 10)

        # Timers
        self.control_timer = self.create_timer(0.25, self.step)  # main loop

        # Goal re-send watchdog (re-send only on stall — no progress for the interval).
        self._goal_last_sent: dict = {ns: 0.0 for ns in self.ns_list}
        self._goal_resend_interval = 12.0  # s with no progress before re-sending
        self._goal_progress_eps = 0.30     # m of closing distance counted as progress
        self._goal_progress: dict = {ns: None for ns in self.ns_list}

        # 多车防撞 — 区域互斥预约：一个取/卸货/充电区同一时刻只允许一台车作为目标。
        self._zone_owner: dict = {}              # zone_name -> ns
        self._home_last_sent: dict = {ns: 0.0 for ns in self.ns_list}
        self._home_resend_interval = 12.0        # s，空闲回待命点的重发间隔（卡住会重试）
        self.standby_tol = 0.6                   # m，到待命点的容差

        # 充电优先让行：去充电的车在路径冲突时享有更高优先级。
        self._yielding: set = set()              # 当前正在让行的车 ns

        # ---- 走廊段路权系统（每拍重建占用 · 原子获取 · 同向共享） ----
        # 井字主干道（针对 amr_vision ±25×±15 大仓 my_map）：3 纵 + 2 横，避开上方两排货架。
        # 纵道走可通行列（左外侧 / 中央货架间隙 / 右外侧）；横道走货架上方与下方空地。
        # 几何随 /fleet/state 发布（corridor_segments），前端按图绘制。
        self.CORRIDOR_SEGMENTS = {
            'corridor_west':      {'x_min': -24.0, 'x_max': -19.0, 'y_min': -13.0, 'y_max': 13.0},
            'corridor_center_ns': {'x_min': -4.0,  'x_max': 1.0,   'y_min': -13.0, 'y_max': 13.0},
            'corridor_east':      {'x_min': 15.0,  'x_max': 20.0,  'y_min': -13.0, 'y_max': 13.0},
            'corridor_north':     {'x_min': -24.0, 'x_max': 20.0,  'y_min': 10.0,  'y_max': 13.0},
            'corridor_south':     {'x_min': -24.0, 'x_max': 20.0,  'y_min': -13.0, 'y_max': -9.0},
        }
        self._segment_occupants: dict = {s: [] for s in self.CORRIDOR_SEGMENTS}
        self._segment_dir: dict = {}            # segment -> 'positive'/'negative'
        self.SEGMENT_CAPACITY = 2               # 每段同向最多车数
        self.PLAN_LOOKAHEAD_M = 4.0             # 规划前瞻距离(m)
        # 让行点：放在各纵道与横道交叉口前的安全处（井字网交叉口）。
        self.WAIT_POINTS = {
            'west_north':   (-21.5,  9.0),
            'west_south':   (-21.5, -8.0),
            'center_north': (-1.5,   9.0),
            'center_south': (-1.5,  -8.0),
            'east_north':   (17.5,   9.0),
            'east_south':   (17.5,  -8.0),
            'north_mid':    (7.0,   11.5),
            'south_mid':    (7.0,  -11.0),
        }
        self._wait_target: dict = {}            # ns -> (x,y) 当前让行等待点目标
        self._wait_progress: dict = {}          # ns -> 让行去等待点的失速看护
        self._yield_blocked_seg: dict = {}      # ns -> 因哪个段而让行
        self._yield_since: dict = {}            # ns -> 进入让行的时刻
        self._yield_min_dwell = 0.5             # s，让行最短保持时长
        self._deadlock_timeout = 8.0            # s，让行久堵兜底
        self._stuck_retreated: set = set()      # 已因卡死后撤过的车（每次卡死仅后撤一次）
        self._stuck_resume_zone: dict = {}      # ns -> 卡死前的任务目标区（恢复时还原）

        # 碰撞检测
        self.collision_dist = 0.55               # m
        self._collisions: list = []              # [{'a','b','d'}]
        self._collision_pairs: set = set()       # 已告警的碰撞对（去抖）
        self.collision_count = 0                 # 累计碰撞事件数

        # ---- Web / dispatch-center interface (consumed by amr_web via rosbridge) ----
        #   OUT: /fleet/state   (std_msgs/String, JSON)  full fleet snapshot, ~3 Hz
        #   IN:  /fleet/add_task(std_msgs/String, JSON)  {"pickup":<zone>,"dropoff":<zone>[,...]}
        self.fleet_state_pub = self.create_publisher(String, '/fleet/state', 10)
        self.create_subscription(String, '/fleet/add_task', self.cb_add_task, 10)
        #   IN:  /fleet/cancel_task(std_msgs/String, JSON)  {"id":<task_id>}
        self.create_subscription(String, '/fleet/cancel_task', self.cb_cancel_task, 10)
        self._added_task_seq = 0
        self.create_timer(0.33, self.publish_fleet_state)

        self.get_logger().info(
            f"FleetManagerAI started. AGVs: {self.ns_list}, tasks in queue: {len(self.task_queue)}")

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
        pct = msg.percentage
        if pct > 1.5:
            pct = pct / 100.0
        pct = max(0.0, min(1.0, pct))
        self.agv[ns].battery = pct

    def cb_batt_float(self, ns, msg: Float32):
        pct = msg.data
        if pct > 1.5:
            pct = pct / 100.0
        pct = max(0.0, min(1.0, pct))
        self.agv[ns].battery = pct

    # ======== Semantic perception (YOLO layer) ========

    def _semantic_callback(self, ns, msg: String):
        """Receive semantic label from camera_detection_node and react."""
        semantic = msg.data
        self.agv[ns].semantic_obstacle = semantic
        self._handle_semantic_change(ns, semantic)

    def _handle_semantic_change(self, ns, semantic):
        """Apply semantic-aware navigation behavior on top of the FSM.

        Only acts during active navigation (TO_PICKUP / TO_DROPOFF):
          person → cancel goal + stop; pallet → slow + let Nav2 reroute;
          clear  → resume the last goal if previously stopped for a person.
        """
        agv = self.agv[ns]
        if agv.state not in ('TO_PICKUP', 'TO_DROPOFF'):
            return

        if semantic == 'person' and not agv.semantic_stop_active:
            agv.semantic_stop_active = True
            self._cancel_nav_goal(ns)
            self.get_logger().warn(f"[FleetAI] {ns}: PERSON detected — STOPPING for safety")

        elif semantic == 'pallet':
            slow = Twist()
            slow.linear.x = 0.1
            slow.angular.z = 0.0
            self.cmd_vel_pub[ns].publish(slow)
            self.get_logger().info(f"[FleetAI] {ns}: PALLET detected — slowing, Nav2 will reroute")

        elif semantic == 'clear' and agv.semantic_stop_active:
            agv.semantic_stop_active = False
            self.get_logger().info(f"[FleetAI] {ns}: path clear — resuming navigation")
            if agv.last_nav_goal is not None:
                agv.last_nav_goal.header.stamp = self.get_clock().now().to_msg()
                self.goal_pub[ns].publish(agv.last_nav_goal)
                self._goal_last_sent[ns] = time.time()

    def _cancel_nav_goal(self, ns):
        """Cancel active Nav2 navigation by sending a goal at the robot's current pose.

        nav2_goal_bridge calls cancelTask() whenever a new (different-position) goal
        arrives, so publishing the current position triggers BT cancellation.
        """
        agv = self.agv[ns]
        if agv.pose is None:
            self.get_logger().warn(
                f"[FleetAI] {ns}: cannot cancel Nav2 goal — pose unknown; "
                "goal re-sending suspended until semantic clears.")
            return
        stop_goal = stamp_now(self)
        stop_goal.pose.position.x = agv.pose[0]
        stop_goal.pose.position.y = agv.pose[1]
        stop_goal.pose.orientation.w = 1.0
        self.goal_pub[ns].publish(stop_goal)
        self._goal_last_sent[ns] = time.time()

    # ======== Main control loop ========

    def step(self):
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
                    if pair not in self._collision_pairs:
                        self.collision_count += 1
                        self.get_logger().error(
                            f"⚠ 碰撞检测：{pair[0]} 与 {pair[1]} 危险接近 (间距 {d:.2f} m < "
                            f"{self.collision_dist} m)！累计 {self.collision_count} 次。")
        self._collision_pairs = active
        self._collisions = cols

    # ======== 路径冲突·优先级让行（碰撞预防） ========

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
        """返回规划路径"前瞻范围内"穿过的走廊段名列表。"""
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
        """判断 AGV 在走廊段内的行进方向：'positive'/'negative'/'unknown'。"""
        yaw = agv.yaw
        if segment_name in ('corridor_east', 'corridor_west', 'corridor_center_ns'):
            return 'positive' if math.sin(yaw) > 0.3 else ('negative' if math.sin(yaw) < -0.3 else 'unknown')
        elif segment_name in ('corridor_north', 'corridor_south'):
            return 'positive' if math.cos(yaw) > 0.3 else ('negative' if math.cos(yaw) < -0.3 else 'unknown')
        return 'unknown'

    def _intended_dir(self, ns, segment_name):
        """车在某走廊段的预期行进方向：优先用规划路径净位移，无则回退航向角。"""
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
        """在本拍正在重建的占用状态下，ns 能否进入 segment_name。"""
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
        """释放 ns 的走廊段占用与让行状态。"""
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
        """解除 ns 的让行状态（不主动下发目标）。"""
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
        """死锁/久堵兜底：让 ns 彻底退出当前走廊段（释放占用 + 后撤到自身段入口）。"""
        for s, occ in self._segment_occupants.items():
            if ns in occ:
                occ.remove(ns)
                if not occ:
                    self._segment_dir.pop(s, None)
        wp = self._retreat_wait_point(ns)
        if wp:
            self._wait_target[ns] = wp
            self.send_goal_xy(ns, wp)
        self._yield_since[ns] = time.time()
        self.get_logger().warn(
            f"[{ns}] 路权久堵({self._deadlock_timeout:.0f}s) → 强制后撤让路。")

    def _vacate_stuck(self, ns):
        """卡死车让出走廊：移除其段占用（不锁段、不逼别人让行），并（每次卡死仅一次）
        强制后撤到等待点。后撤会清空任务目标，故先记下以便恢复时还原。"""
        for s, occ in self._segment_occupants.items():
            if ns in occ:
                occ.remove(ns)
                if not occ:
                    self._segment_dir.pop(s, None)
        if ns not in self._stuck_retreated:
            self._stuck_resume_zone[ns] = self.target_zone_of(ns)
            self._force_retreat(ns)
            self._stuck_retreated.add(ns)
            self.get_logger().warn(f"[{ns}] 导航卡死 → 让出走廊路权并后撤。")

    def _retreat_wait_point(self, ns):
        """选 ns 当前所在段的等待点中、离"被挡段"最远的那个。"""
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
        """为 ns 在指定走廊段找到最近的等待点，避开需跨越当前占用车辆的阻挡点。"""
        agv = self.agv[ns]
        if agv.pose is None:
            return None
        prefix = segment_name
        candidates = [(wp_name, wp_xy) for wp_name, wp_xy in self.WAIT_POINTS.items()
                      if wp_name.startswith(prefix)]
        if not candidates:
            candidates = list(self.WAIT_POINTS.items())

        others = [o for o in self._segment_occupants.get(segment_name, []) if o != ns]
        owner_pose = self.agv[others[0]].pose if (others and self.agv[others[0]].pose) else None

        valid_candidates = []
        if owner_pose:
            is_ns = segment_name in ('corridor_east', 'corridor_west', 'corridor_center_ns')
            for wp_name, wp_xy in candidates:
                if is_ns:
                    y_robot = agv.pose[1]
                    y_owner = owner_pose[1]
                    y_wp = wp_xy[1]
                    if (y_robot < y_owner < y_wp) or (y_wp < y_owner < y_robot):
                        continue
                else:
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
        if best_wp is None:
            for wp_name, wp_xy in valid_candidates:
                d = dist(agv.pose, wp_xy)
                if d < best_d:
                    best_d = d
                    best_wp = wp_xy
        return best_wp

    def apply_traffic_rules(self):
        """走廊段路权（防撞核心）。每拍从零重建段占用，保证原子获取/禁止持有-等待/
        物理在位者优先/隐式优先级抢占。详见 doc/right-of-way-design.md。"""
        # 0) 收集每台车的物理所在段 phys 与需求段 needed(=物理 + 前瞻规划)
        phys, needed, active = {}, {}, []
        for ns in self.ns_list:
            agv = self.agv[ns]
            if agv.pose is None:
                continue
            home = self.home.get(ns)
            at_home = (agv.state == 'IDLE' and home is not None
                       and dist(agv.pose, home) <= self.standby_tol)
            if agv.state == 'CHARGING' or at_home:
                continue
            # 卡死车：让出走廊路权（不锁段、不逼别人让行）并强制后撤一次，避免冻死在
            # 走廊里把别的车堵死。物理避碰交给 Nav2 局部代价地图。
            if self.release_corridor_on_stuck and agv.stuck:
                self._vacate_stuck(ns)
                continue
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

        # 2) 新进入者按 (优先级降序, ns 升序) 原子获取需求段
        order = sorted(active, key=lambda n: (-self.robot_priority(self.agv[n]), n))
        block_of = {}
        for ns in order:
            want_new = [s for s in needed[ns] if ns not in new_occ[s]]
            block = next((s for s in want_new
                          if not self._can_admit(ns, s, new_occ, new_dir)), None)
            if block is None:
                for s in want_new:
                    if not new_occ[s]:
                        new_dir[s] = self._intended_dir(ns, s)
                    new_occ[s].append(ns)
            block_of[ns] = block

        # 3) 提交占用状态
        self._segment_occupants = new_occ
        self._segment_dir = new_dir

        # 4) 久堵兜底：让行过久者后撤；全局"赢家"坚守不退
        yielders = [n for n in active if block_of[n] is not None]
        winner = min(yielders, key=lambda n: (-self.robot_priority(self.agv[n]), n)) if yielders else None

        # 5) 驱动让行/恢复（仅在状态切换时改目标）
        now = time.time()
        for ns in active:
            block = block_of[ns]
            if block is not None:
                if ns not in self._yielding:
                    self._enter_yield(ns, block)
                elif ns != winner and now - self._yield_since.get(ns, now) >= self._deadlock_timeout:
                    self._force_retreat(ns)
                else:
                    self._yield_blocked_seg[ns] = block
                    self._resend_wait_if_stalled(ns)
            elif ns in self._yielding:
                if now - self._yield_since.get(ns, 0.0) >= self._yield_min_dwell:
                    self._clear_yield(ns)
                    self.reissue_goal(ns)

        # 6) 已退出 active 但仍标记让行的车 → 清理
        for ns in list(self._yielding):
            if ns not in active:
                self._clear_yield(ns)

    def reissue_goal(self, ns):
        """让行结束后，按当前状态重新下发目标。语义停车中的车不在此重发。"""
        agv = self.agv[ns]
        if agv.semantic_stop_active:
            return   # 正因行人停车 → 不恢复导航，等语义 clear
        if agv.state == 'TO_CHARGER':
            tz = self.target_zone_of(ns)
            if tz:
                self.nav_to_zone(ns, tz)
        elif agv.state == 'TO_PICKUP' and agv.task:
            self.nav_to_zone(ns, agv.task['pickup'])
        elif agv.state == 'TO_DROPOFF' and agv.task:
            self.nav_to_zone(ns, agv.task['dropoff'])
        elif agv.state == 'IDLE':
            self._home_last_sent[ns] = 0.0

    # ======== Assignment ========

    def assign_tasks(self):
        if not self.task_queue:
            return
        idle = [a for a in self.agv.values() if a.state == 'IDLE' and a.pose is not None
                and a.battery >= self.low_thr and a.ns not in self._yielding]
        if self.require_nav_ready:
            idle = [a for a in idle if a.nav_ready]
        if not idle:
            return

        for agv in idle:
            best = None
            best_d = float('inf')
            for idx, t in enumerate(self.task_queue):
                pz = self.zones.get(t['pickup'])
                if not pz:
                    continue
                if not self.zone_free(t['pickup'], agv.ns) or not self.zone_free(t.get('dropoff'), agv.ns):
                    continue
                d = dist(agv.pose, (pz['cx'], pz['cy']))
                if d < best_d:
                    best_d = d
                    best = (idx, t)
            if best is None:
                continue
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
            self.task_queue.pop(idx)
            break

    # ======== Per-AGV FSM ========

    def run_agv(self, ns):
        agv = self.agv[ns]
        now = time.time()

        # 卡死后撤恢复：已后撤到等待点 → 清卡死、还原任务目标并重发，重新尝试导航。
        # 期间走廊已由 apply_traffic_rules 让出；后撤途中本拍不跑常规 FSM，避免与后撤目标打架。
        if ns in self._stuck_retreated:
            wp = self._wait_target.get(ns)
            if agv.pose is not None and (wp is None or dist(agv.pose, wp) <= self.standby_tol):
                self._stuck_retreated.discard(ns)
                agv.stuck = False
                rz = self._stuck_resume_zone.pop(ns, None)
                if rz is not None:
                    setattr(self, f'_target_zone_{ns}', rz)
                self.reissue_goal(ns)
            return

        # Battery low → head to charger
        if agv.state in ('IDLE', 'TO_PICKUP', 'TO_DROPOFF') and agv.battery < self.low_thr and not agv.on_charger:
            cz_name = self.nearest_charger(agv.pose)
            if cz_name:
                self.release_all(ns)
                self.reserve(cz_name, ns)
                agv.state = 'TO_CHARGER'
                agv.state_since = now
                self.nav_to_zone(ns, cz_name)
                self.get_logger().info(f"[{ns}] Battery low ({agv.battery:.2f}). Heading to charger {cz_name}.")
            return

        # Battery recovered while at charger. Manual charge (operator-dispatched)
        # holds until (near-)full so "去充电区" actually charges; auto charge
        # leaves as soon as it crosses the resume threshold.
        if agv.state in ('CHARGING', 'TO_CHARGER') and agv.on_charger:
            leave_thr = self.full_charge_thr if agv.manual_charge else self.resume_thr
            if agv.battery >= leave_thr:
                self.release_all(ns)
                agv.manual_charge = False
                agv.state = 'IDLE'
                agv.state_since = now
                self.get_logger().info(f"[{ns}] Battery recovered ({agv.battery:.2f}). Back to IDLE.")
                return

        # State logic
        if agv.state == 'IDLE':
            self.set_carrying(ns, False)
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
                self.release(agv.task['pickup'], ns)
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
                self.release_all(ns)
                agv.task = None
                agv.state = 'IDLE'
                agv.state_since = now
                self._home_last_sent[ns] = 0.0
            return

    # ======== Helpers ========

    def goal_xy_of(self, zone_name):
        """Navigation target for a zone: reachable approach point gx,gy if defined, else center."""
        z = self.zones[zone_name]
        return float(z.get('gx', z['cx'])), float(z.get('gy', z['cy']))

    # ---- 区域互斥预约（多车防撞） ----
    def zone_free(self, zone, ns):
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
        self.release_segments(ns)

    def send_goal_xy(self, ns, xy):
        """发布一个非区域的原始目标点（用于返回待命点 / 让行等待点）。"""
        msg = stamp_now(self)
        msg.pose.position.x = float(xy[0])
        msg.pose.position.y = float(xy[1])
        msg.pose.orientation.w = 1.0
        self.goal_pub[ns].publish(msg)
        self.agv[ns].last_nav_goal = msg
        setattr(self, f'_target_zone_{ns}', None)
        self._goal_last_sent[ns] = time.time()
        self._goal_progress[ns] = None

    def nav_to_zone(self, ns, zone_name):
        gx, gy = self.goal_xy_of(zone_name)
        msg = stamp_now(self)
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.orientation.w = 1.0
        # Store goal before publishing so the semantic layer can re-send it on clear.
        self.agv[ns].last_nav_goal = msg
        self.goal_pub[ns].publish(msg)
        setattr(self, f'_target_zone_{ns}', zone_name)
        self._goal_last_sent[ns] = time.time()
        self._goal_progress[ns] = None

    def _resend_goal_if_due(self, ns, zone_name, now):
        """Re-send the goal only if the robot has stalled. Suppressed while a
        semantic person-stop is active (must not re-drive into the person)."""
        agv = self.agv[ns]
        if not zone_name or agv.pose is None:
            return
        if agv.semantic_stop_active:           # 语义停车中：保持停车，不重发
            return
        if ns in self._yielding:               # 让行中：保持停车，不重发
            return
        gx, gy = self.goal_xy_of(zone_name)
        d = dist(agv.pose, (gx, gy))
        prog = self._goal_progress.get(ns)
        if prog is None:
            self._goal_progress[ns] = {'best_d': d, 't': now}
            return
        if d < prog['best_d'] - self._goal_progress_eps:
            prog['best_d'] = d
            prog['t'] = now
            agv.stuck = False
            return
        if now - prog['t'] >= self._goal_resend_interval:
            agv.stuck = True
            self.get_logger().warn(
                f"[{ns}] No progress toward {zone_name} for "
                f"{self._goal_resend_interval:.0f}s (d={d:.2f} m) — re-sending goal.")
            self.nav_to_zone(ns, zone_name)

    def target_zone_of(self, ns):
        return getattr(self, f'_target_zone_{ns}', None)

    def at_zone(self, ns, pose_xy, zone_name):
        if not pose_xy or not zone_name:
            return False
        z = self.zones.get(zone_name)
        if not z:
            return False
        gx, gy = self.goal_xy_of(zone_name)
        if dist(pose_xy, (gx, gy)) <= max(self.reach_dist, 0.5):
            return True
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
        """Append a transport task, or dispatch a manual charge command.

        type=='charge' → send a specific IDLE car to a specific charger now
        (operator override). Otherwise (no type) → queue a pickup→dropoff task.
        """
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f"/fleet/add_task: invalid JSON ({e})")
            return

        if data.get('type') == 'charge':
            self._dispatch_manual_charge(data)
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

    def _dispatch_manual_charge(self, data):
        """操作员手动把指定 IDLE 车派往指定充电桩（立即生效，不入队列）。"""
        ns = data.get('agv')
        charger = data.get('charger')
        if ns not in self.agv:
            self.get_logger().warn(f"/fleet/add_task: charge — unknown agv {ns}")
            return
        if charger not in self.charger_zone_names or charger not in self.zones:
            self.get_logger().warn(
                f"/fleet/add_task: charge — {charger} is not a known charger "
                f"(chargers={self.charger_zone_names})")
            return
        agv = self.agv[ns]
        if agv.state != 'IDLE':
            self.get_logger().warn(
                f"/fleet/add_task: charge — {ns} not IDLE (state={agv.state}); ignored")
            return
        self.release_all(ns)
        self.reserve(charger, ns)
        agv.task = None
        agv.manual_charge = True
        agv.state = 'TO_CHARGER'
        agv.state_since = time.time()
        self.nav_to_zone(ns, charger)
        self.get_logger().info(f"[dispatch] {ns} 手动充电 → {charger}")

    def cb_cancel_task(self, msg: String):
        """按 id 取消任务：队列任务直接移除；执行中任务停车、卸货、释放占用、回 IDLE。"""
        try:
            task_id = str(json.loads(msg.data).get('id', ''))
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f"/fleet/cancel_task: invalid JSON ({e})")
            return
        if not task_id:
            return

        # 1) 排队中（还没派车）→ 直接从队列删除
        before = len(self.task_queue)
        self.task_queue = [t for t in self.task_queue if str(t.get('id')) != task_id]
        if len(self.task_queue) != before:
            self.get_logger().info(f"[dispatch] 取消排队任务 {task_id}")
            return

        # 2) 执行中（已派给某车）→ 停车并复位该车
        for ns, agv in self.agv.items():
            if agv.task and str(agv.task.get('id')) == task_id:
                self._cancel_nav_goal(ns)
                self.release_all(ns)
                self.set_carrying(ns, False)
                agv.task = None
                agv.manual_charge = False
                agv.stuck = False
                self._stuck_retreated.discard(ns)
                self._stuck_resume_zone.pop(ns, None)
                agv.state = 'IDLE'
                agv.state_since = time.time()
                self._home_last_sent[ns] = 0.0
                self.get_logger().info(f"[dispatch] 取消执行中任务 {task_id}（{ns} → IDLE）")
                return

        self.get_logger().warn(f"/fleet/cancel_task: unknown task id {task_id}")

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
                'nav_ready': bool(a.nav_ready),
                'stuck': bool(a.stuck),
                'home_x': round(home[0], 3) if home else None,
                'home_y': round(home[1], 3) if home else None,
                # --- YOLO semantic layer (new for the AI manager) ---
                'semantic': a.semantic_obstacle,
                'semantic_stop': bool(a.semantic_stop_active),
            })
        zones = {
            name: {'cx': float(z['cx']), 'cy': float(z['cy']),
                   'sx': float(z.get('sx', 1.0)), 'sy': float(z.get('sy', 1.0)),
                   'gx': float(z.get('gx', z['cx'])), 'gy': float(z.get('gy', z['cy']))}
            for name, z in self.zones.items()
        }

        # 任务分配总览
        tasks = []
        for ns in self.ns_list:
            a = self.agv[ns]
            if a.task:
                tasks.append({
                    'id': str(a.task.get('id', '?')),
                    'pickup': a.task.get('pickup'),
                    'dropoff': a.task.get('dropoff'),
                    'status': 'assigned',
                    'agv': ns,
                    'agv_state': a.state,
                })
        for t in self.task_queue:
            tasks.append({
                'id': str(t.get('id', '?')),
                'pickup': t.get('pickup'),
                'dropoff': t.get('dropoff'),
                'status': 'queued',
                'agv': None,
                'agv_state': None,
            })
        idle_agvs = [ns for ns in self.ns_list if self.agv[ns].state == 'IDLE']

        # ---- 系统异常总览 ----
        anomalies = []
        for c in self._collisions:
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
            if a.semantic_stop_active:
                anomalies.append({'level': 'warn', 'ns': ns, 'type': '行人停车',
                                  'msg': 'YOLO 检测到行人，已停车等待路径清空'})
            elif a.semantic_obstacle == 'pallet':
                anomalies.append({'level': 'info', 'ns': ns, 'type': '托盘减速',
                                  'msg': 'YOLO 检测到托盘，减速并由 Nav2 绕行'})
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
            'zone_owner': dict(self._zone_owner),
            'segment_owner': {s: lst[0] for s, lst in self._segment_occupants.items() if lst},
            'segment_occupants': {s: list(lst) for s, lst in self._segment_occupants.items() if lst},
            'segment_dir': dict(self._segment_dir),
            'segment_queue': self._segment_waiters(),
            # 走廊几何（前端按图绘制；不同地图自带不同划分）
            'corridor_segments': self.CORRIDOR_SEGMENTS,
            'wait_points': {k: {'x': v[0], 'y': v[1]} for k, v in self.WAIT_POINTS.items()},
            'tasks': tasks,
            'idle_agvs': idle_agvs,
            'yielding': sorted(self._yielding),
            'collisions': self._collisions,
            'collision_count': self.collision_count,
            'anomalies': anomalies,
            'require_nav_ready': self.require_nav_ready,
        }
        self.fleet_state_pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerAI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
