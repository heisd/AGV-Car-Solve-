#!/usr/bin/env python3
"""amr_qt_panel 入口：先起一个空 QMainWindow，后续 Task 逐步填充。"""
import sys


def main(args=None):
    try:
        from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
    except ImportError:
        sys.stderr.write(
            "PyQt5 未安装。请执行: sudo apt install python3-pyqt5\n")
        sys.exit(1)

    app = QApplication(sys.argv)
    win = QMainWindow()
    win.setWindowTitle("AGV 仓库调度中心 (QT)")
    win.setCentralWidget(QLabel("amr_qt_panel 启动成功（骨架）"))
    win.resize(1280, 800)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
