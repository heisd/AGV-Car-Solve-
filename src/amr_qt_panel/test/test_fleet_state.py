import json
from amr_qt_panel.model.fleet_state import FleetState

SAMPLE = json.dumps({
    "stamp": 1.0, "world_name": "warehouse",
    "agvs": [{"ns": "agv1", "x": 1.2, "y": -3.4, "yaw": 0.5, "battery": 0.8,
              "state": "TO_PICKUP", "task": "T1", "carrying": False,
              "on_charger": False, "nav_ready": True, "stuck": False,
              "home_x": 0.0, "home_y": 0.0, "semantic": "person",
              "semantic_stop": True}],
    "zones": {"pickup_zone_A": {"cx": -12.0, "cy": -6.0, "sx": 2.0, "sy": 2.0,
                                "gx": -12.0, "gy": -4.0}},
    "charger_zones": ["charger_1"],
    "tasks": [{"id": "T1", "pickup": "pickup_zone_A", "dropoff": "dropoff_zone_B",
               "status": "assigned", "agv": "agv1", "agv_state": "TO_PICKUP"}],
    "anomalies": [{"level": "warn", "ns": "agv1", "type": "行人停车", "msg": "..."}],
    "queued_tasks": 2, "idle_agvs": [],
    "corridor_segments": {"corridor_east": {"x_min": 15.0, "x_max": 20.0,
                                            "y_min": -13.0, "y_max": 13.0}},
    "segment_owner": {"corridor_east": "agv1"},
})


def test_parse_full():
    fs = FleetState.from_json(SAMPLE)
    assert fs.world_name == "warehouse"
    assert len(fs.agvs) == 1
    a = fs.agvs[0]
    assert a.ns == "agv1" and a.semantic == "person" and a.semantic_stop is True
    assert fs.zones["pickup_zone_A"].gx == -12.0
    assert fs.tasks[0].status == "assigned" and fs.tasks[0].agv == "agv1"
    assert fs.anomalies[0].level == "warn"
    assert fs.queued_tasks == 2
    assert fs.segment_owner["corridor_east"] == "agv1"


def test_parse_empty_tolerant():
    fs = FleetState.from_json("{}")
    assert fs.agvs == [] and fs.zones == {} and fs.queued_tasks == 0


def test_agv_missing_pose():
    fs = FleetState.from_json(json.dumps({"agvs": [{"ns": "agv2"}]}))
    a = fs.agvs[0]
    assert a.ns == "agv2" and a.x is None and a.battery == 0.0
