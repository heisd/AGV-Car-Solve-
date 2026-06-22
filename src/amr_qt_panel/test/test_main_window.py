import json
import pytest
pytest.importorskip("rclpy")  # integration test needs rclpy importable
import rclpy
from amr_qt_panel.ros_bridge import RosBridge
from amr_qt_panel.widgets.main_window import MainWindow


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _emit(b, ns, state):
    b.fleet_state_changed.emit(json.dumps(
        {"agvs": [{"ns": ns, "x": 0.0, "y": 0.0, "state": state}], "zones": {}}))


def test_manual_nav_gated_to_idle(qapp, ros):
    b = RosBridge(); b.start()
    calls = []
    b.publish_goal = lambda ns, x, y, yaw=0.0: calls.append((ns, x, y))
    w = MainWindow(b)
    w._on_robot_selected("agv1")
    _emit(b, "agv1", "TO_PICKUP"); qapp.processEvents()
    w._on_map_clicked(1.0, 2.0)
    assert calls == []                      # busy AGV: manual goal refused
    _emit(b, "agv1", "IDLE"); qapp.processEvents()
    w._on_map_clicked(1.0, 2.0)
    assert calls == [("agv1", 1.0, 2.0)]    # IDLE: goal sent
    w.close()
