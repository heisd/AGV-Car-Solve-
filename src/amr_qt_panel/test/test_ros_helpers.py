import json
import math
from amr_qt_panel.ros_helpers import (add_task_payload, charge_payload,
                                      cancel_task_payload, yaw_to_quat,
                                      camera_topics)


def test_add_task_payload():
    d = json.loads(add_task_payload('pickup_zone_A', 'dropoff_zone_B'))
    assert d == {'pickup': 'pickup_zone_A', 'dropoff': 'dropoff_zone_B'}


def test_charge_payload():
    d = json.loads(charge_payload('agv1', 'charger_1'))
    assert d == {'type': 'charge', 'agv': 'agv1', 'charger': 'charger_1'}


def test_cancel_task_payload():
    d = json.loads(cancel_task_payload('T1'))
    assert d == {'id': 'T1'}


def test_yaw_to_quat_zero():
    x, y, z, w = yaw_to_quat(0.0)
    assert (x, y, z) == (0.0, 0.0, 0.0) and abs(w - 1.0) < 1e-9


def test_yaw_to_quat_halfpi():
    _, _, z, w = yaw_to_quat(math.pi / 2)
    assert abs(z - math.sin(math.pi / 4)) < 1e-9
    assert abs(w - math.cos(math.pi / 4)) < 1e-9


def test_camera_topics():
    img, det = camera_topics('agv2')
    assert img == '/agv2/camera/image_raw'
    assert det == '/agv2/camera/detections'
