import numpy as np
from PyQt5.QtGui import QImage
from amr_qt_panel.widgets.camera_view import CameraView


def _qimage(w=64, h=48):
    arr = np.zeros((h, w, 3), np.uint8)
    return QImage(arr.tobytes(), w, h, w * 3, QImage.Format_RGB888).copy()


def test_set_image_and_detections(qapp):
    v = CameraView()
    v.resize(320, 240)
    v.set_robots(['agv1', 'agv2'])
    v.set_image('agv1', _qimage())
    v.set_detections('agv1', [
        {'class_name': 'person', 'confidence': 0.9, 'bbox': [5, 5, 30, 40]}])
    pm = v.grab()                       # 触发渲染，不应抛异常
    assert pm.width() > 0 and pm.height() > 0


def test_selector_emits(qapp):
    v = CameraView()
    got = []
    v.camera_selected.connect(got.append)
    v.set_robots(['agv1', 'agv2'])
    v._combo.setCurrentIndex(1)         # 选 agv2
    assert got and got[-1] == 'agv2'
