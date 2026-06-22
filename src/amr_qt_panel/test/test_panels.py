import json
from amr_qt_panel.model.fleet_state import FleetState
from amr_qt_panel.widgets.panels import TaskTable, FleetTable, RowTable, AnomalyList

FS = FleetState.from_json(json.dumps({
    "agvs": [{"ns": "agv1", "x": 1.0, "y": 2.0, "battery": 0.55, "state": "TO_PICKUP",
              "carrying": True, "home_x": 0.0, "home_y": 0.0}],
    "tasks": [{"id": "T1", "pickup": "A", "dropoff": "B", "status": "assigned",
               "agv": "agv1", "agv_state": "TO_PICKUP"}],
    "anomalies": [{"level": "error", "ns": "agv1", "type": "碰撞", "msg": "危险"}],
    "corridor_segments": {"corridor_east": {"x_min": 15.0, "x_max": 20.0,
                                            "y_min": -13.0, "y_max": 13.0}},
    "segment_owner": {"corridor_east": "agv1"},
    "segment_queue": {"corridor_east": ["agv2"]},
}))


def test_task_table(qapp):
    t = TaskTable(); t.update_state(FS)
    assert t.rowCount() == 1


def test_task_table_selected_id(qapp):
    t = TaskTable(); t.update_state(FS)
    t.selectRow(0)
    assert t.selected_task_id() == "T1"


def test_task_table_selected_id_none_when_unselected(qapp):
    t = TaskTable(); t.update_state(FS)
    assert t.selected_task_id() is None


def test_fleet_table_select(qapp):
    t = FleetTable(); t.update_state(FS)
    got = []
    t.robot_selected.connect(got.append)
    t.selectRow(0)
    assert got and got[-1] == 'agv1'


def test_row_table(qapp):
    t = RowTable(); t.update_state(FS)
    assert t.rowCount() == 1


def test_anomaly_list(qapp):
    from PyQt5.QtGui import QColor
    a = AnomalyList(); a.update_state(FS)
    assert a.count() == 1
    assert a.item(0).foreground().color() == QColor('#ff6b6b')  # error level
