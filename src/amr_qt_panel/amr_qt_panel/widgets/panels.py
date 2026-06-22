"""信息面板：任务分配 / 车队状态 / 路权走廊 / 系统异常。均从 FleetState 渲染。"""
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (QTableWidget, QTableWidgetItem, QListWidget,
                             QListWidgetItem, QAbstractItemView)
from PyQt5.QtGui import QColor


class _Table(QTableWidget):
    def __init__(self, headers):
        super().__init__(0, len(headers))
        self.setHorizontalHeaderLabels(headers)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)

    def _fill(self, rows):
        self.setRowCount(len(rows))
        for r, cells in enumerate(rows):
            for c, val in enumerate(cells):
                self.setItem(r, c, QTableWidgetItem(str(val)))


class TaskTable(_Table):
    def __init__(self):
        super().__init__(["任务编号", "小车编号", "运行状态", "任务内容"])
        self._ids = []

    def update_state(self, fs):
        self._ids = [t.id for t in fs.tasks]
        rows = [(t.id, t.agv or '-', t.status,
                 f"{t.pickup or '?'} → {t.dropoff or '?'}") for t in fs.tasks]
        self._fill(rows)

    def selected_task_id(self):
        """当前选中行的任务 id；无选中返回 None。"""
        idx = self.currentRow()
        if 0 <= idx < len(self._ids) and self.selectionModel().hasSelection():
            return self._ids[idx]
        return None


class FleetTable(_Table):
    robot_selected = pyqtSignal(str)

    def __init__(self):
        super().__init__(["AGV", "状态", "电量", "载货", "位置(x,y)", "待命点"])
        self._rows = []
        self.itemSelectionChanged.connect(self._on_sel)

    def update_state(self, fs):
        # remember which robot (by ns) was selected, to restore after refill
        prev = None
        idx = self.currentRow()
        if 0 <= idx < len(self._rows):
            prev = self._rows[idx]
        self._rows = [a.ns for a in fs.agvs]
        self.blockSignals(True)
        rows = []
        for a in fs.agvs:
            pos = "-" if a.x is None else f"({a.x:.1f}, {a.y:.1f})"
            home = "-" if a.home_x is None else f"({a.home_x:.1f}, {a.home_y:.1f})"
            bat = "-" if a.battery is None else f"{a.battery*100:.0f}%"
            rows.append((a.ns, a.state, bat,
                         "是" if a.carrying else "否", pos, home))
        self._fill(rows)
        if prev in self._rows:
            self.selectRow(self._rows.index(prev))
        self.blockSignals(False)

    def _on_sel(self):
        idx = self.currentRow()
        if 0 <= idx < len(self._rows):
            self.robot_selected.emit(self._rows[idx])


class RowTable(_Table):
    def __init__(self):
        super().__init__(["走廊段", "占用车", "等待队列", "位置范围(x,y)"])

    def update_state(self, fs):
        rows = []
        for name, seg in fs.corridor_segments.items():
            owner = fs.segment_owner.get(name, '-')
            queue = ", ".join(fs.segment_queue.get(name, []) or []) or '-'
            rng = f"x[{seg['x_min']:.0f},{seg['x_max']:.0f}] " \
                  f"y[{seg['y_min']:.0f},{seg['y_max']:.0f}]"
            rows.append((name, owner, queue, rng))
        self._fill(rows)


class AnomalyList(QListWidget):
    _COLOR = {'error': '#ff6b6b', 'warn': '#ffd166', 'info': '#9fb0c4'}

    def update_state(self, fs):
        self.clear()
        if not fs.anomalies:
            self.addItem(QListWidgetItem("系统正常，无异常…"))
            return
        for an in fs.anomalies:
            it = QListWidgetItem(f"[{an.type}] {an.ns}: {an.msg}")
            it.setForeground(QColor(self._COLOR.get(an.level, '#9fb0c4')))
            self.addItem(it)
