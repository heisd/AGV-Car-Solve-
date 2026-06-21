#!/usr/bin/env python3
"""amr_qt_panel 入口：rclpy.init → QApplication → RosBridge → MainWindow → exec。"""
import sys


def main(args=None):
    try:
        from PyQt5.QtWidgets import QApplication
    except ImportError:
        sys.stderr.write("PyQt5 未安装。请执行: sudo apt install python3-pyqt5\n")
        sys.exit(1)

    import rclpy
    from amr_qt_panel.ros_bridge import RosBridge
    from amr_qt_panel.widgets.main_window import MainWindow

    rclpy.init(args=args)
    app = QApplication(sys.argv)
    bridge = RosBridge(map_ns='agv1')
    bridge.start()
    bridge.set_active_camera('agv1')

    win = MainWindow(bridge)
    win.show()

    try:
        code = app.exec_()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
