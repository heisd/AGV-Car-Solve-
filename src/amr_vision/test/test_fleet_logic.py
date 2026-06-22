# Copyright 2026 GazeboLib
#
# Headless logic tests for fleet_manager_ai.py — manual charge dispatch, task
# cancellation, and stuck-car corridor release. No Gazebo, no executor spin:
# construct the node, stub goal outputs, drive synthetic state, assert decisions.
"""Manual-charge / task-cancel / stuck-vacate logic tests for FleetManagerAI."""

import json

import pytest
import rclpy
from std_msgs.msg import String

from amr_vision.fleet_manager_ai import FleetManagerAI

EAST = 'corridor_east'


@pytest.fixture(scope='module')
def node():
    rclpy.init(args=['--ros-args', '-p', 'robot_namespaces:=["agv1","agv2","agv3"]'])
    n = FleetManagerAI()
    assert n.ns_list == ['agv1', 'agv2', 'agv3']
    n._events = []
    n.send_goal_xy = lambda ns, xy: n._events.append(('wait', ns))
    n.nav_to_zone = lambda ns, z: n._events.append(('nav', ns, z))
    n.reissue_goal = lambda ns: n._events.append(('resume', ns))
    n.zones.setdefault('charger_1', {'cx': -20.0, 'cy': -10.0, 'sx': 1.0, 'sy': 1.0,
                                     'gx': -20.0, 'gy': -10.0})
    n.charger_zone_names = ['charger_1', 'charger_2']
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _reset(n):
    n._events.clear()
    n._segment_occupants = {s: [] for s in n.CORRIDOR_SEGMENTS}
    n._segment_dir = {}
    n._yielding = set()
    n._yield_blocked_seg = {}
    n._wait_target = {}
    n._wait_progress = {}
    n._yield_since = {}
    n._stuck_retreated = set()
    n.task_queue = []
    n._zone_owner = {}
    for ns in n.ns_list:
        a = n.agv[ns]
        a.pose = None
        a.plan_poses = []
        a.state = 'IDLE'
        a.stuck = False
        a.task = None
        a.manual_charge = False
        a.carrying = False
        a.on_charger = False
        a.battery = 1.0


def _car(n, ns, x, y, plan, state='TO_DROPOFF'):
    a = n.agv[ns]
    a.pose = (x, y)
    a.state = state
    a.plan_poses = plan
    a.task = {'id': 't', 'pickup': 'pa', 'dropoff': 'da'}


# ------------------------- Feature 1: manual charge -------------------------

def test_manual_charge_dispatches_idle_car(node):
    _reset(node)
    a = node.agv['agv1']
    a.state = 'IDLE'
    a.pose = (5.0, 0.0)
    node.cb_add_task(String(data=json.dumps(
        {'type': 'charge', 'agv': 'agv1', 'charger': 'charger_1'})))
    assert a.state == 'TO_CHARGER'
    assert a.manual_charge is True
    assert ('nav', 'agv1', 'charger_1') in node._events
    assert node._zone_owner.get('charger_1') == 'agv1'


def test_manual_charge_rejected_when_busy(node):
    _reset(node)
    a = node.agv['agv1']
    a.state = 'TO_PICKUP'
    a.pose = (5.0, 0.0)
    node.cb_add_task(String(data=json.dumps(
        {'type': 'charge', 'agv': 'agv1', 'charger': 'charger_1'})))
    assert a.state == 'TO_PICKUP'          # busy car untouched
    assert a.manual_charge is False


def test_manual_charge_rejected_unknown_charger(node):
    _reset(node)
    a = node.agv['agv1']
    a.state = 'IDLE'
    a.pose = (5.0, 0.0)
    node.cb_add_task(String(data=json.dumps(
        {'type': 'charge', 'agv': 'agv1', 'charger': 'pickup_zone_A'})))
    assert a.state == 'IDLE'               # not a charger → ignored


def test_manual_charge_stays_until_full(node):
    _reset(node)
    a = node.agv['agv1']
    a.state = 'CHARGING'
    a.manual_charge = True
    a.on_charger = True
    a.pose = (-20.0, -10.0)
    a.battery = 0.7                        # > resume_thr but not full
    node.run_agv('agv1')
    assert a.state == 'CHARGING'           # manual: do NOT leave early
    a.battery = 1.0
    node.run_agv('agv1')
    assert a.state == 'IDLE'
    assert a.manual_charge is False


def test_auto_charge_still_leaves_at_resume_thr(node):
    _reset(node)
    a = node.agv['agv1']
    a.state = 'CHARGING'
    a.manual_charge = False
    a.on_charger = True
    a.pose = (-20.0, -10.0)
    a.battery = node.resume_thr + 0.01     # auto charge returns at resume_thr
    node.run_agv('agv1')
    assert a.state == 'IDLE'


def test_transport_task_still_works(node):
    """Backward compat: a no-type payload is still a transport task."""
    _reset(node)
    node.zones.setdefault('pickup_zone_A', {'cx': -12.0, 'cy': -6.0, 'sx': 1.0, 'sy': 1.0})
    node.zones.setdefault('dropoff_zone_B', {'cx': 12.0, 'cy': 10.0, 'sx': 1.0, 'sy': 1.0})
    node.cb_add_task(String(data=json.dumps(
        {'pickup': 'pickup_zone_A', 'dropoff': 'dropoff_zone_B'})))
    assert len(node.task_queue) == 1
    assert node.task_queue[0]['pickup'] == 'pickup_zone_A'


# ------------------------- Feature 2: task cancel -------------------------

def test_cancel_queued_task(node):
    _reset(node)
    node.task_queue = [{'id': 'T9', 'pickup': 'pa', 'dropoff': 'da'}]
    node.cb_cancel_task(String(data=json.dumps({'id': 'T9'})))
    assert node.task_queue == []


def test_cancel_in_progress_task(node):
    _reset(node)
    a = node.agv['agv1']
    a.state = 'TO_PICKUP'
    a.pose = (5.0, 0.0)
    a.carrying = True
    a.task = {'id': 'T5', 'pickup': 'pa', 'dropoff': 'da'}
    node.cb_cancel_task(String(data=json.dumps({'id': 'T5'})))
    assert a.task is None
    assert a.state == 'IDLE'
    assert a.carrying is False


def test_cancel_unknown_id_noop(node):
    _reset(node)
    node.task_queue = [{'id': 'T1', 'pickup': 'pa', 'dropoff': 'da'}]
    node.cb_cancel_task(String(data=json.dumps({'id': 'NOPE'})))
    assert len(node.task_queue) == 1


# ------------------------- Feature 3: stuck vacates corridor -------------------------

def test_stuck_car_releases_corridor(node):
    _reset(node)
    _car(node, 'agv1', 17.5, 0.0, [(17.5, 0.0)])
    node.agv['agv1'].stuck = True
    _car(node, 'agv2', 17.5, -6.0, [(17.5, -6.0), (17.5, -3.0), (17.5, 0.0)])
    node.apply_traffic_rules()
    assert 'agv1' not in node._segment_occupants[EAST]   # stuck car vacated
    assert 'agv2' in node._segment_occupants[EAST]       # other car proceeds


def test_stuck_car_force_retreats_once(node):
    _reset(node)
    _car(node, 'agv1', 17.5, 0.0, [(17.5, 0.0)])
    node.agv['agv1'].stuck = True
    node.apply_traffic_rules()
    assert ('wait', 'agv1') in node._events              # retreat goal sent
    node._events.clear()
    node.apply_traffic_rules()
    assert ('wait', 'agv1') not in node._events          # not re-sent every tick


def test_stuck_retreat_recovers_at_wait_point(node):
    """撤到等待点后：清卡死、移出 retreated 集、重发任务目标重试。"""
    _reset(node)
    a = node.agv['agv1']
    _car(node, 'agv1', 17.5, 0.0, [(17.5, 0.0)], state='TO_DROPOFF')
    a.stuck = True
    node.apply_traffic_rules()                           # vacate + retreat
    assert 'agv1' in node._stuck_retreated
    assert node._wait_target.get('agv1') is not None
    a.pose = node._wait_target['agv1']                   # 模拟撤到等待点
    node._events.clear()
    node.run_agv('agv1')                                 # 恢复
    assert 'agv1' not in node._stuck_retreated
    assert a.stuck is False
    assert ('resume', 'agv1') in node._events            # reissue_goal 被调用


def test_stuck_retreating_skips_fsm_until_wait_point(node):
    """后撤途中（未到等待点）本拍不跑常规 FSM。"""
    _reset(node)
    a = node.agv['agv1']
    _car(node, 'agv1', 17.5, 0.0, [(17.5, 0.0)], state='TO_DROPOFF')
    a.stuck = True
    node.apply_traffic_rules()                           # retreat → _wait_target set
    a.pose = (17.5, 1.0)                                 # 还没到等待点
    node._events.clear()
    node.run_agv('agv1')
    assert 'agv1' in node._stuck_retreated               # 仍在后撤
    assert ('resume', 'agv1') not in node._events        # 未恢复
