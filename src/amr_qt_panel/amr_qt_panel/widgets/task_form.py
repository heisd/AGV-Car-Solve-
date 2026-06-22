"""下发任务表单：取货/卸货下拉（排除充电区）→ dispatch 信号；
另含「去充电区」小节：选车（仅 IDLE）+ 选充电桩 → charge_dispatch 信号。"""
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (QWidget, QFormLayout, QComboBox, QPushButton,
                             QLabel, QVBoxLayout, QFrame)


def _refill(combo, items):
    """重填下拉，尽量保留当前选中项。"""
    cur = combo.currentText()
    combo.blockSignals(True)
    combo.clear()
    combo.addItems(items)
    if cur in items:
        combo.setCurrentText(cur)
    combo.blockSignals(False)


class TaskForm(QWidget):
    dispatch = pyqtSignal(str, str)
    charge_dispatch = pyqtSignal(str, str)   # (agv, charger)

    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        form = QFormLayout()
        self._pickup = QComboBox()
        self._dropoff = QComboBox()
        form.addRow("取货区", self._pickup)
        form.addRow("卸货区", self._dropoff)
        root.addLayout(form)
        self._btn = QPushButton("下发任务")
        self._btn.clicked.connect(self._on_click)
        root.addWidget(self._btn)
        self._queue = QLabel("待分配任务队列：-")
        root.addWidget(self._queue)

        # ---- 去充电区小节 ----
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        root.addWidget(line)
        charge_form = QFormLayout()
        self._charge_agv = QComboBox()
        self._charge_zone = QComboBox()
        charge_form.addRow("车辆", self._charge_agv)
        charge_form.addRow("充电桩", self._charge_zone)
        root.addLayout(charge_form)
        self._charge_btn = QPushButton("去充电区")
        self._charge_btn.clicked.connect(self._on_charge_click)
        root.addWidget(self._charge_btn)

    def update_zones(self, fs):
        names = [n for n in fs.zones.keys() if n not in fs.charger_zones]
        _refill(self._pickup, names)
        _refill(self._dropoff, names)
        self._queue.setText(f"待分配任务队列：{fs.queued_tasks}")

        # 去充电区：车辆下拉只列空闲车，充电桩下拉列所有充电桩
        _refill(self._charge_agv, list(fs.idle_agvs))
        _refill(self._charge_zone, list(fs.charger_zones))
        self._charge_btn.setEnabled(
            self._charge_agv.count() > 0 and self._charge_zone.count() > 0)

    def _on_click(self):
        p, d = self._pickup.currentText(), self._dropoff.currentText()
        if p and d:
            self.dispatch.emit(p, d)

    def _on_charge_click(self):
        a, c = self._charge_agv.currentText(), self._charge_zone.currentText()
        if a and c:
            self.charge_dispatch.emit(a, c)
