import json
from types import SimpleNamespace
from amr_qt_panel.model.fleet_state import FleetState
from amr_qt_panel.widgets.task_form import TaskForm
from amr_qt_panel.widgets.log_list import LogList

FS = FleetState.from_json(json.dumps({
    "zones": {"pickup_zone_A": {"cx": -12.0, "cy": -6.0},
              "dropoff_zone_B": {"cx": 12.0, "cy": 10.0},
              "charger_1": {"cx": -20.0, "cy": -10.0}},
    "charger_zones": ["charger_1"], "queued_tasks": 3,
}))


def test_task_form_zones_exclude_charger(qapp):
    f = TaskForm(); f.update_zones(FS)
    items = [f._pickup.itemText(i) for i in range(f._pickup.count())]
    assert "pickup_zone_A" in items and "charger_1" not in items


def test_task_form_dispatch(qapp):
    f = TaskForm(); f.update_zones(FS)
    got = []
    f.dispatch.connect(lambda p, d: got.append((p, d)))
    f._pickup.setCurrentText("pickup_zone_A")
    f._dropoff.setCurrentText("dropoff_zone_B")
    f._btn.click()
    assert got == [("pickup_zone_A", "dropoff_zone_B")]


def test_log_info_toggle(qapp):
    lg = LogList()
    info = SimpleNamespace(level=20, name="nav", msg="hello")
    err = SimpleNamespace(level=40, name="ctrl", msg="boom")
    lg.append_log(info)     # 默认不显示 INFO
    lg.append_log(err)
    assert lg._list.count() == 1
    lg._show_info.setChecked(True)
    lg.append_log(info)
    assert lg._list.count() == 2
