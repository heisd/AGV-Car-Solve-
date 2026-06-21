"""MainWindow：阶段一仅含相机；阶段二/三会加入地图与各表。"""
from PyQt5.QtWidgets import QMainWindow

from amr_qt_panel.widgets.camera_view import CameraView


class MainWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self._bridge = bridge
        self.setWindowTitle("AGV 仓库调度中心 (QT)")
        self.resize(1280, 800)

        self._camera = CameraView()
        self.setCentralWidget(self._camera)

        # 信号接线（GUI 线程槽）
        bridge.image_changed.connect(self._camera.set_image)
        bridge.detections_changed.connect(self._camera.set_detections)
        bridge.fleet_state_changed.connect(self._on_fleet_state)
        self._camera.camera_selected.connect(bridge.set_active_camera)

    def _on_fleet_state(self, raw):
        # 从 fleet/state 动态发现机器人，填充相机下拉
        import json
        try:
            agvs = json.loads(raw).get('agvs', [])
        except (ValueError, TypeError):
            return
        self._camera.set_robots([a.get('ns') for a in agvs if a.get('ns')])

    def closeEvent(self, ev):
        self._bridge.shutdown()
        super().closeEvent(ev)
