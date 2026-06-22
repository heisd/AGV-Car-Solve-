"""/rosout 日志：INFO 开关 + 清空。"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QListWidget,
                             QCheckBox, QPushButton, QLabel)

_LEVEL = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


class LogList(QWidget):
    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        bar = QHBoxLayout()
        bar.addWidget(QLabel("异常日志 (/rosout)"))
        self._show_info = QCheckBox("含 INFO")
        bar.addWidget(self._show_info)
        clear = QPushButton("清空")
        clear.clicked.connect(lambda: self._list.clear())
        bar.addWidget(clear)
        bar.addStretch(1)
        root.addLayout(bar)
        self._list = QListWidget()
        root.addWidget(self._list)

    def append_log(self, msg):
        level = getattr(msg, 'level', 20)
        if level <= 20 and not self._show_info.isChecked():
            return
        name = getattr(msg, 'name', '')
        text = getattr(msg, 'msg', '')
        self._list.addItem(f"[{_LEVEL.get(level, level)}] {name}: {text}")
        if self._list.count() > 500:
            self._list.takeItem(0)
