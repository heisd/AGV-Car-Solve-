#!/usr/bin/env python3
import os
import math
import yaml
import time
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
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

        self.ns_list = list(self.get_parameter('robot_namespaces').value)
        self.low_thr = float(self.get_parameter('battery_low_threshold').value)
        self.resume_thr = float(self.get_parameter('battery_resume_threshold').value)
        self.reach_dist = float(self.get_parameter('goal_reach_dist').value)
        self.batt_topic_type = str(self.get_parameter('battery_topic_type').value)
        self.charger_zone_names = list(self.get_parameter('charger_zone_names').value)

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
        self._home_sent: dict = {ns: False for ns in self.ns_list}
        self.standby_tol = 0.6                   # m，到待命点的容差

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

    def cb_on_charger(self, ns, msg: Bool):
        self.agv[ns].on_charger = bool(msg.data)

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

        # Assign tasks to idle AGVs
        self.assign_tasks()

    # ======== Assignment ========

    def assign_tasks(self):
        if not self.task_queue:
            return
        # candidates: idle AGVs with battery >= low_thr
        idle = [a for a in self.agv.values() if a.state == 'IDLE' and a.pose is not None and a.battery >= self.low_thr]
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
            self._home_sent[agv.ns] = False
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
            # 空闲且不在待命点 → 回各自独立的待命点，清空作业区、分散车辆（防撞核心）
            home = self.home.get(ns)
            if home and agv.pose is not None:
                if dist(agv.pose, home) > self.standby_tol:
                    if not self._home_sent[ns]:
                        self.send_goal_xy(ns, home)
                        self._home_sent[ns] = True
                        self.get_logger().info(
                            f"[{ns}] Idle — 返回待命点 ({home[0]:.1f}, {home[1]:.1f}).")
                else:
                    self._home_sent[ns] = False   # 已到位；被挤走后可再次回位
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
                self._home_sent[ns] = False    # 触发下一拍返回待命点
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
        gx, gy = self.goal_xy_of(zone_name)
        d = dist(agv.pose, (gx, gy))
        prog = self._goal_progress.get(ns)
        if prog is None:                       # first observation after a fresh send
            self._goal_progress[ns] = {'best_d': d, 't': now}
            return
        if d < prog['best_d'] - self._goal_progress_eps:
            prog['best_d'] = d                 # closing in — refresh watchdog
            prog['t'] = now
            return
        if now - prog['t'] >= self._goal_resend_interval:
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
            })
        zones = {
            name: {'cx': float(z['cx']), 'cy': float(z['cy']),
                   'sx': float(z.get('sx', 1.0)), 'sy': float(z.get('sy', 1.0)),
                   'gx': float(z.get('gx', z['cx'])), 'gy': float(z.get('gy', z['cy']))}
            for name, z in self.zones.items()
        }
        payload = {
            'stamp': time.time(),
            'agvs': agvs,
            'queued_tasks': len(self.task_queue),
            'zones': zones,
            'charger_zones': self.charger_zone_names,
            'zone_owner': dict(self._zone_owner),   # zone_name -> ns（区域预约，防撞可视化）
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