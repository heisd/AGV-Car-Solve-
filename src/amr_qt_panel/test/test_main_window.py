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


def _emit_task(b, task_id):
    b.fleet_state_changed.emit(json.dumps({
        "agvs": [], "zones": {},
        "tasks": [{"id": task_id, "pickup": "A", "dropoff": "B",
                   "status": "queued"}],
    }))


def test_charge_dispatch_wired(qapp, ros):
    b = RosBridge(); b.start()
    calls = []
    b.publish_charge = lambda agv, charger: calls.append((agv, charger))
    w = MainWindow(b)
    w._form.charge_dispatch.emit("agv2", "charger_1")
    assert calls == [("agv2", "charger_1")]
    w.close()


def test_cancel_confirmed(qapp, ros, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    b = RosBridge(); b.start()
    calls = []
    b.publish_cancel = lambda task_id: calls.append(task_id)
    w = MainWindow(b)
    _emit_task(b, "T1"); qapp.processEvents()
    w._tasks.selectRow(0)
    monkeypatch.setattr(QMessageBox, "question", lambda *a, **k: QMessageBox.Yes)
    w._on_cancel_clicked()
    assert calls == ["T1"]
    w.close()


def test_cancel_declined(qapp, ros, monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    b = RosBridge(); b.start()
    calls = []
    b.publish_cancel = lambda task_id: calls.append(task_id)
    w = MainWindow(b)
    _emit_task(b, "T1"); qapp.processEvents()
    w._tasks.selectRow(0)
    monkeypatch.setattr(QMessageBox, "question", lambda *a, **k: QMessageBox.No)
    w._on_cancel_clicked()
    assert calls == []                       # declined → no publish
    w.close()


def test_cancel_noop_without_selection(qapp, ros):
    b = RosBridge(); b.start()
    calls = []
    b.publish_cancel = lambda task_id: calls.append(task_id)
    w = MainWindow(b)
    w._on_cancel_clicked()                   # nothing selected
    assert calls == []
    w.close()


def test_bridge_publish_charge_cancel_callable(qapp, ros):
    b = RosBridge(); b.start()
    b.publish_charge("agv1", "charger_1")    # must not raise
    b.publish_cancel("T1")                   # must not raise
    b.shutdown()
