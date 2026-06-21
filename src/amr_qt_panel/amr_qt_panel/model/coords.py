"""世界坐标(m) ↔ 画布像素，逐行移植 amr_web/app.js 的变换。"""

DEFAULT_VIEW = 7.6   # app.js:16
PAD = 24             # app.js:125
DEFAULT_SIZE = 600   # app.js canvas.width


class Transform:
    def __init__(self, size: int = DEFAULT_SIZE, view: float = DEFAULT_VIEW):
        self.size = size
        self.pad = PAD
        self.span = size - 2 * PAD
        self.view = view
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.locked = False

    def _base_x(self, wx):
        return self.pad + ((wx + self.view) / (2 * self.view)) * self.span

    def _base_y(self, wy):
        return self.pad + ((self.view - wy) / (2 * self.view)) * self.span

    def _base_l(self, m):
        return (m / (2 * self.view)) * self.span

    def to_px(self, wx, wy):
        return (self._base_x(wx) * self.zoom + self.pan_x,
                self._base_y(wy) * self.zoom + self.pan_y)

    def to_len(self, m):
        return self._base_l(m) * self.zoom

    def from_px(self, px, py):
        wx = ((((px - self.pan_x) / self.zoom) - self.pad) / self.span) \
            * (2 * self.view) - self.view
        wy = self.view - (((((py - self.pan_y) / self.zoom) - self.pad)
                           / self.span) * (2 * self.view))
        return (wx, wy)

    def calibrate_from_map(self, origin_x, origin_y, width, height, resolution):
        """app.js:213-215——按 /map 范围定标 VIEW（仅一次，受 locked 保护）。"""
        if self.locked:
            return
        half_x = max(abs(origin_x), abs(origin_x + width * resolution))
        half_y = max(abs(origin_y), abs(origin_y + height * resolution))
        self.view = max(half_x, half_y, 1.0) * 1.04
        self.locked = True
