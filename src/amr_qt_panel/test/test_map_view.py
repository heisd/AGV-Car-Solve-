import json
import numpy as np
from types import SimpleNamespace
from amr_qt_panel.model.fleet_state import FleetState
from amr_qt_panel.widgets.map_view import MapView


def _grid():
    info = SimpleNamespace(resolution=0.1, width=20, height=20,
                           origin=SimpleNamespace(
                               position=SimpleNamespace(x=-1.0, y=-1.0, z=0.0)))
    data = [0] * (20 * 20)
    return SimpleNamespace(info=info, data=data)


def test_render_smoke(qapp):
    v = MapView()
    v.resize(600, 600)
    fs = FleetState.from_json(json.dumps({
        "agvs": [{"ns": "agv1", "x": 0.0, "y": 0.0, "yaw": 0.0, "battery": 0.7,
                  "state": "IDLE"}],
        "zones": {"pickup_zone_A": {"cx": -2.0, "cy": -1.0, "sx": 1.0, "sy": 1.0}},
    }))
    v.set_map('agv1', _grid())
    v.set_fleet_state(fs)
    v.set_path('agv1', [(0.0, 0.0), (1.0, 1.0)])
    pm = v.grab()                       # 不应抛异常
    assert pm.width() > 0


def test_click_emits_world(qapp):
    v = MapView()
    v.resize(600, 600)
    got = []
    v.map_clicked.connect(lambda x, y: got.append((x, y)))
    v._emit_click_at(300, 300)          # 测试钩子：用画布中心像素
    assert got and len(got[0]) == 2


def test_zoom_anchors_to_cursor(qapp):
    v = MapView()
    v.resize(600, 600)
    px, py = 220.0, 160.0
    before = v._t.from_px(px, py)
    v.zoom_at(px, py, 1.5)
    after = v._t.from_px(px, py)
    assert abs(after[0] - before[0]) < 1e-6 and abs(after[1] - before[1]) < 1e-6
