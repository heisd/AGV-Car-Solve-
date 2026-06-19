# Copyright 2026 GazeboLib
#
# Headless logic tests for the走廊段路权 (right-of-way) core in fleet_manager.py.
# Drive FleetManager.apply_traffic_rules() with synthetic poses/plans (no Gazebo,
# no executor spin) and assert the safety/liveness invariants of the v3.0 redesign.
"""Right-of-way (走廊段占用制) logic tests for FleetManager."""

import time

import pytest
import rclpy

from amr_description.fleet_manager import FleetManager

EAST = 'corridor_east'
NORTH = 'corridor_north'


@pytest.fixture(scope='module')
def node():
    """Bring up a 3-AGV FleetManager with goal outputs stubbed to a recorder."""
    rclpy.init(args=['--ros-args', '-p', 'robot_namespaces:=["agv1","agv2","agv3"]'])
    n = FleetManager()
    assert n.ns_list == ['agv1', 'agv2', 'agv3']
    n._events = []
    n.send_goal_xy = lambda ns, xy: n._events.append(('wait', ns))
    n.nav_to_zone = lambda ns, z: n._events.append(('nav', ns))
    n.reissue_goal = lambda ns: n._events.append(('resume', ns))
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
    for ns in n.ns_list:
        n.agv[ns].pose = None
        n.agv[ns].plan_poses = []
        n.agv[ns].state = 'IDLE'


def _car(n, ns, x, y, plan, yaw=0.0, state='TO_DROPOFF'):
    a = n.agv[ns]
    a.pose = (x, y)
    a.yaw = yaw
    a.state = state
    a.plan_poses = plan
    a.task = {'id': 't', 'pickup': 'pa', 'dropoff': 'da'}


def test_head_on_one_yields(node):
    """Opposing cars in one corridor: exactly one proceeds, the other yields."""
    _reset(node)
    _car(node, 'agv1', 5.5, -6.8, [(5.5, -6.8), (5.5, -3.0), (5.5, 0.0)])   # +y
    _car(node, 'agv2', 5.5, 6.8, [(5.5, 6.8), (5.5, 3.0), (5.5, 0.0)])      # -y
    node.apply_traffic_rules()
    assert node._segment_occupants[EAST] == ['agv1']   # deterministic winner (ns order)
    assert 'agv2' in node._yielding
    assert 'agv1' not in node._yielding


def test_winner_is_deterministic(node):
    """The head-on winner must not depend on dict/set iteration order."""
    winners = set()
    for _ in range(8):
        _reset(node)
        _car(node, 'agv1', 5.5, -6.8, [(5.5, -6.8), (5.5, -3.0), (5.5, 0.0)])
        _car(node, 'agv2', 5.5, 6.8, [(5.5, 6.8), (5.5, 3.0), (5.5, 0.0)])
        node.apply_traffic_rules()
        winners.add(node._segment_occupants[EAST][0])
    assert winners == {'agv1'}


def test_same_direction_convoy_shares(node):
    """Two same-direction cars share one segment (no needless yield)."""
    _reset(node)
    _car(node, 'agv1', 5.5, -6.8, [(5.5, -6.8), (5.5, -3.0), (5.5, 0.0)])
    _car(node, 'agv2', 5.5, -7.6, [(5.5, -7.6), (5.5, -6.5), (5.5, -3.0)])
    node.apply_traffic_rules()
    assert sorted(node._segment_occupants[EAST]) == ['agv1', 'agv2']
    assert len(node._yielding) == 0


def test_capacity_overflow_third_yields(node):
    """A same-direction third car yields once the segment hits SEGMENT_CAPACITY."""
    _reset(node)
    _car(node, 'agv1', 5.5, -6.8, [(5.5, -6.8), (5.5, -3.0), (5.5, 0.0)])
    _car(node, 'agv2', 5.5, -7.6, [(5.5, -7.6), (5.5, -6.5), (5.5, -3.0)])
    _car(node, 'agv3', 5.5, -8.4, [(5.5, -8.4), (5.5, -7.0), (5.5, -5.0)])
    node.apply_traffic_rules()
    assert len(node._segment_occupants[EAST]) == node.SEGMENT_CAPACITY
    assert 'agv3' in node._yielding


def test_corner_no_hold_and_wait(node):
    """At a corner, a blocked car keeps its physical segment but does NOT grab the blocked one."""
    _reset(node)
    _car(node, 'agv1', 5.5, 2.0, [(5.5, 2.0), (5.5, 5.0), (5.5, 5.6)], yaw=1.5708)
    _car(node, 'agv2', 3.0, 5.5, [(3.0, 5.5), (0.0, 5.5), (-3.0, 5.5)], yaw=3.14159)
    node.apply_traffic_rules()
    assert 'agv1' in node._segment_occupants[EAST]      # incumbent kept
    assert node._segment_occupants[NORTH] == ['agv2']   # did NOT acquire blocked seg
    assert 'agv1' in node._yielding


def test_mutual_deadlock_backstop(node):
    """A genuine 2-car mutual block: the non-winner force-retreats, the winner holds."""
    _reset(node)
    _car(node, 'agv1', 5.5, 2.0, [(5.5, 2.0), (5.5, 5.0), (5.5, 5.5), (4.0, 5.5)])
    _car(node, 'agv2', 3.0, 5.5, [(3.0, 5.5), (5.0, 5.5), (5.5, 5.5), (5.5, 3.0)])
    node.apply_traffic_rules()
    assert node._yielding == {'agv1', 'agv2'}           # real mutual block
    node._yield_since['agv2'] = time.time() - (node._deadlock_timeout + 2)
    node._events.clear()
    node.apply_traffic_rules()
    assert time.time() - node._yield_since.get('agv2', 0) < 1.0   # retreat reset its timer
    assert 'agv2' not in node._segment_occupants[NORTH]           # loser released segment
    assert 'agv1' in node._segment_occupants[EAST]                # winner held


def test_payload_shape_frontend_compatible(node):
    """segment_owner stays a {seg: ns-string} map; segment_queue stays {seg: list}."""
    _reset(node)
    _car(node, 'agv1', 5.5, -6.8, [(5.5, -6.8), (5.5, -3.0), (5.5, 0.0)])
    _car(node, 'agv2', 5.5, 6.8, [(5.5, 6.8), (5.5, 3.0), (5.5, 0.0)])
    node.apply_traffic_rules()
    owner = {s: lst[0] for s, lst in node._segment_occupants.items() if lst}
    queue = node._segment_waiters()
    assert all(isinstance(v, str) for v in owner.values())
    assert all(isinstance(v, list) for v in queue.values())
