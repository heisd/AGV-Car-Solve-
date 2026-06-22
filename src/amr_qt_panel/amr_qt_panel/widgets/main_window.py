"""MainWindow：左地图 + 右侧栏（阶段二：地图+相机）。"""
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout

from amr_qt_panel.model.fleet_state import FleetState
from amr_qt_panel.widgets.camera_view import CameraView
from amr_qt_panel.widgets.map_view import MapView


class MainWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self._bridge = bridge
        self.setWindowTitle("AGV 仓库调度中心 (QT)")
        self.resize(1400, 860)
        self._selected = None

        central = QWidget()
        root = QHBoxLayout(central)
        self._map = MapView()
        root.addWidget(self._map, 3)

        side = QVBoxLayout()
        self._camera = CameraView()
        side.addWidget(self._camera, 1)
        root.addLayout(side, 2)
        self.setCentralWidget(central)

        # 接线
        bridge.image_changed.connect(self._camera.set_image)
        bridge.detections_changed.connect(self._camera.set_detections)
        bridge.map_changed.connect(self._map.set_map)
        bridge.path_changed.connect(self._map.set_path)
        bridge.fleet_state_changed.connect(self._on_fleet_state_raw)
        self._camera.camera_selected.connect(bridge.set_active_camera)
        self._map.map_clicked.connect(self._on_map_clicked)

    def _on_fleet_state_raw(self, raw):
        try:
            fs = FleetState.from_json(raw)
        except (ValueError, TypeError):
            return
        self._map.set_fleet_state(fs)
        robots = [a.ns for a in fs.agvs if a.ns]
        self._camera.set_robots(robots)
        for ns in robots:
            self._bridge.ensure_path_sub(ns)

    def _on_map_clicked(self, wx, wy):
        if self._selected:
            self._bridge.publish_goal(self._selected, wx, wy)

    def closeEvent(self, ev):
        self._bridge.shutdown()
        super().closeEvent(ev)
