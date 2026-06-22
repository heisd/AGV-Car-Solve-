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


FS_CHARGE = FleetState.from_json(json.dumps({
    "zones": {"pickup_zone_A": {"cx": -12.0, "cy": -6.0},
              "charger_1": {"cx": -20.0, "cy": -10.0},
              "charger_2": {"cx": 20.0, "cy": -10.0}},
    "charger_zones": ["charger_1", "charger_2"],
    "idle_agvs": ["agv1", "agv2"],
}))


def test_task_form_charge_options(qapp):
    """充电小节：车辆下拉只列空闲车，充电桩下拉只列充电桩。"""
    f = TaskForm(); f.update_zones(FS_CHARGE)
    agvs = [f._charge_agv.itemText(i) for i in range(f._charge_agv.count())]
    chargers = [f._charge_zone.itemText(i) for i in range(f._charge_zone.count())]
    assert agvs == ["agv1", "agv2"]
    assert chargers == ["charger_1", "charger_2"]


def test_task_form_charge_dispatch(qapp):
    f = TaskForm(); f.update_zones(FS_CHARGE)
    got = []
    f.charge_dispatch.connect(lambda a, c: got.append((a, c)))
    f._charge_agv.setCurrentText("agv2")
    f._charge_zone.setCurrentText("charger_2")
    f._charge_btn.click()
    assert got == [("agv2", "charger_2")]


def test_task_form_charge_btn_disabled_when_no_idle(qapp):
    """没有空闲车时充电按钮禁用。"""
    fs = FleetState.from_json(json.dumps({
        "zones": {"charger_1": {"cx": -20.0, "cy": -10.0}},
        "charger_zones": ["charger_1"], "idle_agvs": [],
    }))
    f = TaskForm(); f.update_zones(fs)
    assert not f._charge_btn.isEnabled()


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
