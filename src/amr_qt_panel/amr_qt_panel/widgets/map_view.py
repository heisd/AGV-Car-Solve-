"""MapView：QPainter 重画占用栅格 + 静态图元 + Nav2 路径 + AGV；滚轮缩放、点击导航。"""
import math
import numpy as np
from PyQt5.QtCore import Qt, pyqtSignal, QPointF
from PyQt5.QtGui import QPainter, QColor, QPen, QImage, QPixmap, QPolygonF
from PyQt5.QtWidgets import QWidget

from amr_qt_panel.model.coords import Transform

AGV_COLORS = ['#3da9fc', '#ffa733', '#3ddc84', '#c87cff', '#ff7ca8']


class MapView(QWidget):
    map_clicked = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(600, 600)
        self._t = Transform()
        self._fs = None
        self._map_bitmap = None      # QImage 缓存
        self._map_meta = None        # (origin_x, origin_y, w, h, res)
        self._paths = {}             # ns -> [(x,y)]
        self._selected = None
        self._drag = None
        self._ns_index = {}

    # ---- 槽 ----
    def set_fleet_state(self, fs):
        self._fs = fs
        for a in fs.agvs:
            if a.ns and a.ns not in self._ns_index:
                self._ns_index[a.ns] = len(self._ns_index)
        self.update()

    def set_map(self, ns, grid):
        info = grid.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        self._t.calibrate_from_map(ox, oy, w, h, res)
        # 占用栅格 -> QImage（numpy 向量化；左下角=origin，需上下翻转）
        data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
        gray = np.full((h, w), 235, dtype=np.uint8)    # 空闲=浅
        gray[data < 0] = 130                           # 未知=灰
        gray[data >= 65] = 30                          # 占据=深
        rgb = np.ascontiguousarray(
            np.repeat(np.flipud(gray)[:, :, None], 3, axis=2))
        img = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        self._map_bitmap = img
        self._map_meta = (ox, oy, w, h, res)
        self.update()

    def set_path(self, ns, pts):
        self._paths[ns] = pts
        self.update()

    def set_selected_robot(self, ns):
        self._selected = ns

    # ---- 渲染 ----
    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor('#11161d'))
        self._draw_map(p)
        if self._fs is not None:
            self._draw_static(p, self._fs)
            self._draw_paths(p)
            self._draw_agvs(p, self._fs)

    def _draw_map(self, p):
        if self._map_bitmap is None or self._map_meta is None:
            return
        ox, oy, w, h, res = self._map_meta
        x0, y0 = self._t.to_px(ox, oy + h * res)   # 左上角
        dw = self._t.to_len(w * res)
        dh = self._t.to_len(h * res)
        p.setRenderHint(QPainter.SmoothPixmapTransform, False)
        p.drawPixmap(int(x0), int(y0), int(dw), int(dh),
                     QPixmap.fromImage(self._map_bitmap))

    def _rect_world(self, p, cx, cy, sw, sh, color, fill=True):
        x, y = self._t.to_px(cx - sw / 2, cy + sh / 2)
        w, h = self._t.to_len(sw), self._t.to_len(sh)
        if fill:
            p.fillRect(int(x), int(y), int(w), int(h), QColor(color))
        else:
            p.setPen(QPen(QColor(color)))
            p.drawRect(int(x), int(y), int(w), int(h))

    def _draw_static(self, p, fs):
        # 走廊段（淡描边）
        for name, seg in fs.corridor_segments.items():
            cx = (seg['x_min'] + seg['x_max']) / 2
            cy = (seg['y_min'] + seg['y_max']) / 2
            sw = seg['x_max'] - seg['x_min']
            sh = seg['y_max'] - seg['y_min']
            self._rect_world(p, cx, cy, sw, sh, '#2a3850', fill=False)
        # 区域（货架/取货/卸货/充电）。颜色按 app.js drawStatic 分类着色补全。
        for name, z in fs.zones.items():
            color = '#6741d9' if name in fs.charger_zones else '#274060'
            self._rect_world(p, z.cx, z.cy, z.sx, z.sy, color, fill=True)
            x, y = self._t.to_px(z.cx - z.sx / 2, z.cy + z.sy / 2)
            p.setPen(QColor('#cdd8e6'))
            p.drawText(int(x) + 2, int(y) - 3, name)

    def _draw_paths(self, p):
        for ns, pts in self._paths.items():
            if not pts:
                continue
            i = self._ns_index.get(ns, 0)
            pen = QPen(QColor(AGV_COLORS[i % len(AGV_COLORS)]))
            pen.setWidth(2)
            pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            poly = QPolygonF([QPointF(*self._t.to_px(x, y)) for x, y in pts])
            p.drawPolyline(poly)

    def _draw_agvs(self, p, fs):
        for i, a in enumerate(fs.agvs):
            if a.x is None or a.y is None:
                continue
            cx, cy = self._t.to_px(a.x, a.y)
            color = QColor(AGV_COLORS[self._ns_index.get(a.ns, i) % len(AGV_COLORS)])
            s = 9.0
            pts = [(0, -s), (s * 0.7, s * 0.7), (-s * 0.7, s * 0.7)]
            ca, sa = math.cos(a.yaw), math.sin(a.yaw)
            poly = QPolygonF([
                QPointF(cx + dx * ca - dy * sa, cy + dx * sa + dy * ca)
                for dx, dy in pts])
            p.setBrush(color)
            p.setPen(QPen(QColor('#0b0e13')))
            p.drawPolygon(poly)
            p.setPen(QColor('#cdd8e6'))
            p.drawText(int(cx) + 8, int(cy), a.ns)

    # ---- 交互 ----
    def zoom_at(self, px, py, factor):
        """Zoom by factor while keeping the world point under pixel (px,py) fixed."""
        wx, wy = self._t.from_px(px, py)
        self._t.zoom *= factor
        nx, ny = self._t.to_px(wx, wy)
        self._t.pan_x += px - nx
        self._t.pan_y += py - ny
        self.update()

    def wheelEvent(self, ev):
        factor = 1.1 if ev.angleDelta().y() > 0 else 1 / 1.1
        self.zoom_at(ev.x(), ev.y(), factor)

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._emit_click_at(ev.x(), ev.y())

    def _emit_click_at(self, px, py):
        wx, wy = self._t.from_px(px, py)
        self.map_clicked.emit(float(wx), float(wy))
