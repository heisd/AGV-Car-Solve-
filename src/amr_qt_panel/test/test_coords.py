from amr_qt_panel.model.coords import Transform


def test_round_trip():
    t = Transform(size=600, view=7.6)
    for wx, wy in [(0, 0), (3.5, -2.1), (-7, 7), (6.6, -6.6)]:
        px, py = t.to_px(wx, wy)
        rx, ry = t.from_px(px, py)
        assert abs(rx - wx) < 1e-6 and abs(ry - wy) < 1e-6


def test_round_trip_zoom_pan():
    t = Transform()
    t.zoom, t.pan_x, t.pan_y = 1.7, 40.0, -25.0
    px, py = t.to_px(2.0, -1.0)
    rx, ry = t.from_px(px, py)
    assert abs(rx - 2.0) < 1e-6 and abs(ry + 1.0) < 1e-6


def test_calibrate_from_map():
    t = Transform()
    t.calibrate_from_map(-10.0, -10.0, 200, 200, 0.1)   # extent ±10
    assert abs(t.view - 10.0 * 1.04) < 1e-6
    assert t.locked is True


def test_calibrate_respects_lock():
    t = Transform()
    t.locked = True
    t.calibrate_from_map(-10.0, -10.0, 200, 200, 0.1)
    assert t.view == 7.6
