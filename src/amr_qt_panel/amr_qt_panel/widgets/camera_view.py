"""CameraView：单路大画面 + 切车下拉 + YOLO 框叠加。"""
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QColor, QPixmap
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QLabel


class _Canvas(QWidget):
    """实际画图的内嵌控件：缩放显示 QImage + 叠加检测框。"""
    def __init__(self):
        super().__init__()
        self._image = None         # QImage
        self._dets = []            # list[dict]
        self.setMinimumSize(320, 240)

    def update_image(self, qimage):
        self._image = qimage
        self.update()

    def update_dets(self, dets):
        self._dets = dets or []
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor('#11161d'))
        if self._image is None or self._image.isNull():
            p.setPen(QColor('#9fb0c4'))
            p.drawText(self.rect(), Qt.AlignCenter, "等待相机话题…")
            return
        # 等比缩放居中
        pm = QPixmap.fromImage(self._image)
        scaled = pm.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        ox = (self.width() - scaled.width()) // 2
        oy = (self.height() - scaled.height()) // 2
        p.drawPixmap(ox, oy, scaled)
        # 叠框：bbox 是源图像素坐标，按缩放比换算
        sx = scaled.width() / self._image.width()
        sy = scaled.height() / self._image.height()
        pen = QPen(QColor('#3ddc84'))
        pen.setWidth(2)
        p.setPen(pen)
        for d in self._dets:
            bb = d.get('bbox')
            if not bb or len(bb) != 4:
                continue
            x1, y1, x2, y2 = bb
            rx, ry = ox + x1 * sx, oy + y1 * sy
            rw, rh = (x2 - x1) * sx, (y2 - y1) * sy
            p.drawRect(int(rx), int(ry), int(rw), int(rh))
            label = f"{d.get('class_name', '?')} {d.get('confidence', 0):.2f}"
            p.drawText(int(rx), max(0, int(ry) - 4), label)


class CameraView(QWidget):
    camera_selected = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._active = None
        lay = QVBoxLayout(self)
        bar = QHBoxLayout()
        bar.addWidget(QLabel("相机视频流"))
        self._combo = QComboBox()
        self._combo.currentTextChanged.connect(self._on_pick)
        bar.addWidget(self._combo)
        bar.addStretch(1)
        lay.addLayout(bar)
        self._canvas = _Canvas()
        lay.addWidget(self._canvas, 1)

    def _on_pick(self, ns):
        if ns and ns != self._active:
            self._active = ns
            self.camera_selected.emit(ns)

    def set_robots(self, robots):
        cur = self._combo.currentText()
        self._combo.blockSignals(True)
        self._combo.clear()
        self._combo.addItems(robots)
        self._combo.blockSignals(False)
        if cur in robots:
            self._combo.setCurrentText(cur)
        elif robots and self._active is None:
            self._combo.setCurrentIndex(0)

    def set_image(self, ns, qimage):
        if ns == self._active or self._active is None:
            self._canvas.update_image(qimage)

    def set_detections(self, ns, dets):
        if ns == self._active or self._active is None:
            self._canvas.update_dets(dets)
