"""下发任务表单：取货/卸货下拉（排除充电区）→ dispatch 信号。"""
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (QWidget, QFormLayout, QComboBox, QPushButton,
                             QLabel, QVBoxLayout)


class TaskForm(QWidget):
    dispatch = pyqtSignal(str, str)

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

    def update_zones(self, fs):
        names = [n for n in fs.zones.keys() if n not in fs.charger_zones]
        for combo in (self._pickup, self._dropoff):
            cur = combo.currentText()
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(names)
            if cur in names:
                combo.setCurrentText(cur)
            combo.blockSignals(False)
        self._queue.setText(f"待分配任务队列：{fs.queued_tasks}")

    def _on_click(self):
        p, d = self._pickup.currentText(), self._dropoff.currentText()
        if p and d:
            self.dispatch.emit(p, d)
