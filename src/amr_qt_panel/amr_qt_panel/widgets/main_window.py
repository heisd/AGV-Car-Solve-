"""MainWindow：左地图 + 右 QScrollArea（相机/异常/下发/任务/车队/路权/日志）。"""
from PyQt5.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
                             QScrollArea, QLabel)

from amr_qt_panel.model.fleet_state import FleetState
from amr_qt_panel.widgets.camera_view import CameraView
from amr_qt_panel.widgets.map_view import MapView
from amr_qt_panel.widgets.panels import TaskTable, FleetTable, RowTable, AnomalyList
from amr_qt_panel.widgets.task_form import TaskForm
from amr_qt_panel.widgets.log_list import LogList


def _titled(title, w):
    box = QWidget()
    lay = QVBoxLayout(box)
    lay.setContentsMargins(0, 0, 0, 0)
    lay.addWidget(QLabel(title))
    lay.addWidget(w)
    return box


class MainWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self._bridge = bridge
        self.setWindowTitle("AGV 仓库调度中心 (QT)")
        self.resize(1480, 900)
        self._selected = None

        central = QWidget()
        root = QHBoxLayout(central)
        self._map = MapView()
        root.addWidget(_titled("仓库实时地图", self._map), 3)

        # 右侧滚动栏
        side_host = QWidget()
        side = QVBoxLayout(side_host)
        self._camera = CameraView()
        self._anomaly = AnomalyList()
        self._form = TaskForm()
        self._tasks = TaskTable()
        self._fleet = FleetTable()
        self._row = RowTable()
        self._log = LogList()
        side.addWidget(self._camera, 2)
        side.addWidget(_titled("系统异常", self._anomaly))
        side.addWidget(_titled("下发任务", self._form))
        side.addWidget(_titled("任务分配情况", self._tasks))
        side.addWidget(_titled("车队状态", self._fleet))
        side.addWidget(_titled("路权与走廊状态", self._row))
        side.addWidget(self._log)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(side_host)
        root.addWidget(scroll, 2)
        self.setCentralWidget(central)

        # 接线
        bridge.image_changed.connect(self._camera.set_image)
        bridge.detections_changed.connect(self._camera.set_detections)
        bridge.map_changed.connect(self._map.set_map)
        bridge.path_changed.connect(self._map.set_path)
        bridge.fleet_state_changed.connect(self._on_fleet_state_raw)
        bridge.log_received.connect(self._log.append_log)
        self._camera.camera_selected.connect(bridge.set_active_camera)
        self._map.map_clicked.connect(self._on_map_clicked)
        self._fleet.robot_selected.connect(self._on_robot_selected)
        self._form.dispatch.connect(bridge.publish_add_task)

    def _on_fleet_state_raw(self, raw):
        try:
            fs = FleetState.from_json(raw)
        except (ValueError, TypeError):
            return
        self._map.set_fleet_state(fs)
        self._tasks.update_state(fs)
        self._fleet.update_state(fs)
        self._row.update_state(fs)
        self._anomaly.update_state(fs)
        self._form.update_zones(fs)
        robots = [a.ns for a in fs.agvs if a.ns]
        self._camera.set_robots(robots)
        for ns in robots:
            self._bridge.ensure_path_sub(ns)

    def _on_robot_selected(self, ns):
        self._selected = ns
        self._map.set_selected_robot(ns)

    def _on_map_clicked(self, wx, wy):
        if self._selected:
            self._bridge.publish_goal(self._selected, wx, wy)

    def closeEvent(self, ev):
        self._bridge.shutdown()
        super().closeEvent(ev)
