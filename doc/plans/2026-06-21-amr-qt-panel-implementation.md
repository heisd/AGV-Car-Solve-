# amr_qt_panel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增 ROS 2 包 `amr_qt_panel`——一个 PyQt5 原生桌面面板，全量复刻 web 面板（地图/任务分配/车队/路权/异常/日志/下发任务），并通过直接订阅 ROS 2 图像话题提供稳定的 YOLO 标注视频流。

**Architecture:** `RosBridge(QObject)` 在后台线程跑 rclpy 执行器，订阅回调只 `emit` Qt 信号；GUI 线程的槽函数更新控件（方案 A，线程+信号）。各 widget 从 `FleetState` 模型/信号渲染。视频直接订 `/{ns}/camera/image_raw`（不走 web_video_server），YOLO 框由 QPainter 叠加。

**Tech Stack:** Python 3.10、ROS 2 Humble（rclpy）、PyQt5、numpy；pytest（`QT_QPA_PLATFORM=offscreen` 跑无头 widget 测试）。

参考 spec：`doc/specs/2026-06-21-amr-qt-panel-design.md`。

## Global Constraints

- 包名 `amr_qt_panel`，类型 `ament_python`；入口 console_script：`amr_qt_panel = amr_qt_panel.app:main`。
- **不修改** `amr_vision` / `amr_web` 任何文件；本包只作消费端。
- 面板侧 **不依赖** `cv2` / `cv_bridge` / `ultralytics` / `web_video_server`。
- 线程铁律：**只有 GUI 线程槽函数能触碰 QWidget**；后台 ROS 线程只 `emit` 信号。`Image→QImage` 转换在后台线程完成后再 emit。
- QoS：`/{ns}/map` 用 **RELIABLE + TRANSIENT_LOCAL，depth 1**；`/{ns}/camera/image_raw` 用 **`qos_profile_sensor_data`（BEST_EFFORT）**；其余默认 reliable keep_last 10。
- 坐标常量逐字移植 `app.js`：`VIEW` 初始 `7.6`、`PAD=24`、`SIZE=600`、`SPAN=SIZE-2*PAD`；定标 `VIEW=max(halfX,halfY,1)*1.04`。
- 默认显示 `agv1` 的 `/map`，可由 launch 参数 `map_ns` 覆盖。
- 每个提交信息以 `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>` 结尾。
- 纯逻辑测试不依赖已 source 的 ROS（用 `types.SimpleNamespace` 假冒 msg）。

---

## 阶段一：包骨架 + 稳定视频（核心价值先交付）

### Task 1: 包骨架（可 build、可起空窗口）

**Files:**
- Create: `src/amr_qt_panel/package.xml`
- Create: `src/amr_qt_panel/setup.py`
- Create: `src/amr_qt_panel/setup.cfg`
- Create: `src/amr_qt_panel/resource/amr_qt_panel`（空文件）
- Create: `src/amr_qt_panel/amr_qt_panel/__init__.py`（空）
- Create: `src/amr_qt_panel/amr_qt_panel/app.py`
- Create: `src/amr_qt_panel/test/conftest.py`

**Interfaces:**
- Produces: `amr_qt_panel.app:main()` 入口；pytest `qapp` 会话级 fixture（offscreen QApplication）。

- [ ] **Step 1: 写 package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>amr_qt_panel</name>
  <version>0.1.0</version>
  <description>Native PyQt5 desktop operator panel for the AMR fleet (direct ROS 2 image subscription + map + task allocation).</description>
  <maintainer email="jvf36276@gmail.com">heisd</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>rcl_interfaces</exec_depend>
  <exec_depend>python3-pyqt5</exec_depend>
  <exec_depend>python3-numpy</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: 写 setup.py**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_qt_panel'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heisd',
    maintainer_email='jvf36276@gmail.com',
    description='Native PyQt5 desktop operator panel for the AMR fleet.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr_qt_panel = amr_qt_panel.app:main',
        ],
    },
)
```

- [ ] **Step 3: 写 setup.cfg**

```ini
[develop]
script_dir=$base/lib/amr_qt_panel
[install]
install_scripts=$base/lib/amr_qt_panel
```

- [ ] **Step 4: 写最小 app.py（空窗口）**

```python
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
```

- [ ] **Step 5: 写 resource marker 与 __init__.py**

`resource/amr_qt_panel` 与 `amr_qt_panel/__init__.py` 均为空文件：
```bash
mkdir -p src/amr_qt_panel/resource src/amr_qt_panel/amr_qt_panel src/amr_qt_panel/test
: > src/amr_qt_panel/resource/amr_qt_panel
: > src/amr_qt_panel/amr_qt_panel/__init__.py
```

- [ ] **Step 6: 写 test/conftest.py（offscreen QApplication fixture）**

```python
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import pytest
from PyQt5.QtWidgets import QApplication


@pytest.fixture(scope='session')
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app
```

- [ ] **Step 7: 构建并验证**

Run:
```bash
cd /home/li/GazeboLib && colcon build --packages-select amr_qt_panel
```
Expected: `Finished <<< amr_qt_panel`，无报错。

Run:
```bash
source install/setup.bash && ros2 pkg list | grep amr_qt_panel
```
Expected: 输出 `amr_qt_panel`。

- [ ] **Step 8: Commit**

```bash
git add src/amr_qt_panel
git commit -m "feat(amr_qt_panel): scaffold ament_python package with empty window

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: `image_convert.py` —— sensor_msgs/Image → QImage（纯逻辑，TDD）

**Files:**
- Create: `src/amr_qt_panel/amr_qt_panel/model/__init__.py`（空）
- Create: `src/amr_qt_panel/amr_qt_panel/model/image_convert.py`
- Test: `src/amr_qt_panel/test/test_image_convert.py`

**Interfaces:**
- Produces: `image_to_qimage(msg) -> QImage`，`msg` 需有 `height,width,encoding,step,data`；支持 `rgb8/bgr8/mono8`，其它抛 `ValueError`。返回值已 `.copy()` 脱离原缓冲。

- [ ] **Step 1: 写失败测试**

```python
from types import SimpleNamespace
import numpy as np
import pytest
from PyQt5.QtGui import QImage
from amr_qt_panel.model.image_convert import image_to_qimage


def _msg(arr, encoding):
    h, w = arr.shape[0], arr.shape[1]
    ch = 1 if arr.ndim == 2 else arr.shape[2]
    return SimpleNamespace(height=h, width=w, encoding=encoding,
                           step=w * ch, data=arr.tobytes())


def test_rgb8():
    arr = np.zeros((4, 5, 3), np.uint8)
    arr[..., 0] = 255
    img = image_to_qimage(_msg(arr, 'rgb8'))
    assert img.width() == 5 and img.height() == 4
    assert img.format() == QImage.Format_RGB888
    assert img.pixelColor(0, 0).red() == 255


def test_mono8():
    arr = np.full((3, 3), 7, np.uint8)
    img = image_to_qimage(_msg(arr, 'mono8'))
    assert img.width() == 3 and img.height() == 3


def test_unsupported():
    arr = np.zeros((2, 2, 4), np.uint8)
    with pytest.raises(ValueError):
        image_to_qimage(_msg(arr, 'rgba8'))
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_image_convert.py -v`
Expected: FAIL（`ModuleNotFoundError: amr_qt_panel.model.image_convert`）。

- [ ] **Step 3: 实现 image_convert.py**

```python
"""sensor_msgs/Image -> QImage，纯 numpy 实现（不依赖 cv2 / cv_bridge）。"""
from PyQt5.QtGui import QImage


def image_to_qimage(msg) -> QImage:
    """支持 rgb8 / bgr8 / mono8；其它编码抛 ValueError。返回值已 copy()。"""
    enc = msg.encoding
    w, h, step = msg.width, msg.height, msg.step
    buf = bytes(msg.data)

    if enc == 'rgb8':
        img = QImage(buf, w, h, step, QImage.Format_RGB888)
    elif enc == 'bgr8':
        fmt = getattr(QImage, 'Format_BGR888', None)
        if fmt is not None:
            img = QImage(buf, w, h, step, fmt)
        else:
            img = QImage(buf, w, h, step, QImage.Format_RGB888).rgbSwapped()
    elif enc == 'mono8':
        img = QImage(buf, w, h, step, QImage.Format_Grayscale8)
    else:
        raise ValueError(f"unsupported image encoding: {enc}")

    return img.copy()  # 脱离 buf，避免悬垂引用
```

- [ ] **Step 4: 跑测试确认通过**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_image_convert.py -v`
Expected: 3 passed。

- [ ] **Step 5: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/model/__init__.py src/amr_qt_panel/amr_qt_panel/model/image_convert.py src/amr_qt_panel/test/test_image_convert.py
git commit -m "feat(amr_qt_panel): add Image->QImage converter (numpy, no cv2)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: `ros_bridge.py` —— ROS 边界（线程/信号/订阅/发布）

**Files:**
- Create: `src/amr_qt_panel/amr_qt_panel/ros_bridge.py`
- Test: `src/amr_qt_panel/test/test_ros_helpers.py`（纯 helper 单测）

**Interfaces:**
- Produces:
  - 纯 helper：`add_task_payload(pickup, dropoff) -> str`（JSON）、`yaw_to_quat(yaw) -> (x,y,z,w)`、`camera_topics(ns) -> (image_topic, det_topic)`。
  - `RosBridge(QObject)` 信号：`fleet_state_changed(object)`、`image_changed(str, object)`、`detections_changed(str, object)`、`map_changed(str, object)`、`path_changed(str, object)`、`log_received(object)`。
  - 方法：`start()`、`shutdown()`、`set_active_camera(ns)`、`set_map_ns(ns)`、`publish_goal(ns, x, y, yaw=0.0)`、`publish_add_task(pickup, dropoff)`。

- [ ] **Step 1: 写 helper 失败测试**

```python
import json
import math
from amr_qt_panel.ros_bridge import add_task_payload, yaw_to_quat, camera_topics


def test_add_task_payload():
    d = json.loads(add_task_payload('pickup_zone_A', 'dropoff_zone_B'))
    assert d == {'pickup': 'pickup_zone_A', 'dropoff': 'dropoff_zone_B'}


def test_yaw_to_quat_zero():
    x, y, z, w = yaw_to_quat(0.0)
    assert (x, y, z) == (0.0, 0.0, 0.0) and abs(w - 1.0) < 1e-9


def test_yaw_to_quat_halfpi():
    _, _, z, w = yaw_to_quat(math.pi / 2)
    assert abs(z - math.sin(math.pi / 4)) < 1e-9
    assert abs(w - math.cos(math.pi / 4)) < 1e-9


def test_camera_topics():
    img, det = camera_topics('agv2')
    assert img == '/agv2/camera/image_raw'
    assert det == '/agv2/camera/detections'
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd src/amr_qt_panel && python -m pytest test/test_ros_helpers.py -v`
Expected: FAIL（`ModuleNotFoundError: amr_qt_panel.ros_bridge`）。

- [ ] **Step 3: 实现 ros_bridge.py**

```python
"""RosBridge：唯一 ROS 边界。后台线程跑执行器，回调只 emit Qt 信号。"""
import json
import math
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from std_msgs.msg import String

from PyQt5.QtCore import QObject, pyqtSignal

from amr_qt_panel.model.image_convert import image_to_qimage


# ----- 纯 helper（可无 ROS 单测）-----
def add_task_payload(pickup: str, dropoff: str) -> str:
    return json.dumps({'pickup': pickup, 'dropoff': dropoff})


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def camera_topics(ns: str):
    return (f'/{ns}/camera/image_raw', f'/{ns}/camera/detections')


_MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class RosBridge(QObject):
    fleet_state_changed = pyqtSignal(object)
    image_changed = pyqtSignal(str, object)        # (ns, QImage)
    detections_changed = pyqtSignal(str, object)   # (ns, list[dict])
    map_changed = pyqtSignal(str, object)          # (ns, OccupancyGrid)
    path_changed = pyqtSignal(str, object)         # (ns, list[(x,y)])
    log_received = pyqtSignal(object)              # rcl_interfaces/Log

    def __init__(self, map_ns: str = 'agv1'):
        super().__init__()
        self._node = Node('amr_qt_panel')
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._active_cam = None
        self._cam_subs = []          # 当前相机的订阅（切车时销毁）
        self._path_subs = {}         # ns -> sub
        self._map_ns = map_ns
        self._map_sub = None

        # 常驻订阅
        self._node.create_subscription(
            String, '/fleet/state', self._on_fleet_state, 10)
        self._node.create_subscription(
            Log, '/rosout', self._on_log, 10)
        self._set_map_sub(map_ns)

        # 发布器缓存：ns -> goal_pose publisher
        self._goal_pubs = {}
        self._add_task_pub = self._node.create_publisher(
            String, '/fleet/add_task', 10)

    # ---- 生命周期 ----
    def start(self):
        self._thread.start()

    def _spin(self):
        try:
            self._exec.spin()
        except Exception:
            pass

    def shutdown(self):
        try:
            self._exec.shutdown()
        finally:
            self._node.destroy_node()

    # ---- 订阅管理 ----
    def _set_map_sub(self, ns):
        if self._map_sub is not None:
            self._node.destroy_subscription(self._map_sub)
        self._map_ns = ns
        self._map_sub = self._node.create_subscription(
            OccupancyGrid, f'/{ns}/map',
            lambda m, ns=ns: self.map_changed.emit(ns, m), _MAP_QOS)

    def set_map_ns(self, ns):
        self._set_map_sub(ns)

    def set_active_camera(self, ns):
        for s in self._cam_subs:
            self._node.destroy_subscription(s)
        self._cam_subs = []
        self._active_cam = ns
        img_topic, det_topic = camera_topics(ns)
        self._cam_subs.append(self._node.create_subscription(
            Image, img_topic,
            lambda m, ns=ns: self.image_changed.emit(ns, image_to_qimage(m)),
            qos_profile_sensor_data))
        self._cam_subs.append(self._node.create_subscription(
            String, det_topic,
            lambda m, ns=ns: self._on_detections(ns, m), 10))

    def ensure_path_sub(self, ns):
        if ns in self._path_subs:
            return
        self._path_subs[ns] = self._node.create_subscription(
            Path, f'/{ns}/plan',
            lambda m, ns=ns: self.path_changed.emit(
                ns, [(p.pose.position.x, p.pose.position.y) for p in m.poses]),
            10)

    # ---- 回调 ----
    def _on_fleet_state(self, msg):
        self.fleet_state_changed.emit(msg.data)

    def _on_detections(self, ns, msg):
        try:
            dets = json.loads(msg.data).get('detections', [])
        except (ValueError, TypeError):
            dets = []
        self.detections_changed.emit(ns, dets)

    def _on_log(self, msg):
        self.log_received.emit(msg)

    # ---- 发布 ----
    def publish_goal(self, ns, x, y, yaw=0.0):
        pub = self._goal_pubs.get(ns)
        if pub is None:
            pub = self._node.create_publisher(PoseStamped, f'/{ns}/goal_pose', 10)
            self._goal_pubs[ns] = pub
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        pub.publish(msg)

    def publish_add_task(self, pickup, dropoff):
        self._add_task_pub.publish(String(data=add_task_payload(pickup, dropoff)))
```

- [ ] **Step 4: 跑 helper 测试确认通过**

Run: `cd src/amr_qt_panel && python -m pytest test/test_ros_helpers.py -v`
Expected: 4 passed。（注意：本测试只导入纯 helper，导入 `ros_bridge` 需 ROS 环境已 source；若未 source，单独把 helper 拆到 `ros_helpers.py` 亦可。执行时请先 `source install/setup.bash`。）

- [ ] **Step 5: 集成冒烟（需 source ROS）**

Run:
```bash
source install/setup.bash
python -c "
import rclpy; from amr_qt_panel.ros_bridge import RosBridge
rclpy.init(); b=RosBridge(); b.start(); b.set_active_camera('agv1')
import time; time.sleep(0.5); b.shutdown(); rclpy.shutdown(); print('OK')
"
```
Expected: 打印 `OK`，无异常。

- [ ] **Step 6: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/ros_bridge.py src/amr_qt_panel/test/test_ros_helpers.py
git commit -m "feat(amr_qt_panel): add RosBridge (executor thread + Qt signals)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: `camera_view.py` —— 大画面 + 切车 + YOLO 叠框

**Files:**
- Create: `src/amr_qt_panel/amr_qt_panel/widgets/__init__.py`（空）
- Create: `src/amr_qt_panel/amr_qt_panel/widgets/camera_view.py`
- Test: `src/amr_qt_panel/test/test_camera_view.py`

**Interfaces:**
- Consumes: `image_changed(ns, QImage)`、`detections_changed(ns, list[dict])`（每个 dict 含 `class_name, confidence, bbox=[x1,y1,x2,y2]`）。
- Produces: `CameraView(QWidget)`；信号 `camera_selected(str)`（切车时发出，供上层调用 `RosBridge.set_active_camera`）；槽 `set_image(ns, qimage)`、`set_detections(ns, dets)`、`set_robots(list[str])`。

- [ ] **Step 1: 写 offscreen 冒烟测试**

```python
import numpy as np
from PyQt5.QtGui import QImage
from amr_qt_panel.widgets.camera_view import CameraView


def _qimage(w=64, h=48):
    arr = np.zeros((h, w, 3), np.uint8)
    return QImage(arr.tobytes(), w, h, w * 3, QImage.Format_RGB888).copy()


def test_set_image_and_detections(qapp):
    v = CameraView()
    v.resize(320, 240)
    v.set_robots(['agv1', 'agv2'])
    v.set_image('agv1', _qimage())
    v.set_detections('agv1', [
        {'class_name': 'person', 'confidence': 0.9, 'bbox': [5, 5, 30, 40]}])
    pm = v.grab()                       # 触发渲染，不应抛异常
    assert pm.width() > 0 and pm.height() > 0


def test_selector_emits(qapp):
    v = CameraView()
    got = []
    v.camera_selected.connect(got.append)
    v.set_robots(['agv1', 'agv2'])
    v._combo.setCurrentIndex(1)         # 选 agv2
    assert got and got[-1] == 'agv2'
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_camera_view.py -v`
Expected: FAIL（`ModuleNotFoundError`）。

- [ ] **Step 3: 实现 camera_view.py**

```python
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
```

- [ ] **Step 4: 跑测试确认通过**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_camera_view.py -v`
Expected: 2 passed。

- [ ] **Step 5: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/widgets/__init__.py src/amr_qt_panel/amr_qt_panel/widgets/camera_view.py src/amr_qt_panel/test/test_camera_view.py
git commit -m "feat(amr_qt_panel): add CameraView with YOLO box overlay

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: 阶段一总装（app.py 接 RosBridge + CameraView + launch）

**Files:**
- Modify: `src/amr_qt_panel/amr_qt_panel/app.py`
- Create: `src/amr_qt_panel/amr_qt_panel/widgets/main_window.py`
- Create: `src/amr_qt_panel/launch/qt_panel.launch.py`

**Interfaces:**
- Consumes: `RosBridge`、`CameraView`。
- Produces: `MainWindow(QMainWindow)`（阶段一只放 CameraView，后续 Task 扩展）；可运行的 launch。

- [ ] **Step 1: 写 main_window.py（阶段一版）**

```python
"""MainWindow：阶段一仅含相机；阶段二/三会加入地图与各表。"""
from PyQt5.QtWidgets import QMainWindow

from amr_qt_panel.widgets.camera_view import CameraView


class MainWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self._bridge = bridge
        self.setWindowTitle("AGV 仓库调度中心 (QT)")
        self.resize(1280, 800)

        self._camera = CameraView()
        self.setCentralWidget(self._camera)

        # 信号接线（GUI 线程槽）
        bridge.image_changed.connect(self._camera.set_image)
        bridge.detections_changed.connect(self._camera.set_detections)
        bridge.fleet_state_changed.connect(self._on_fleet_state)
        self._camera.camera_selected.connect(bridge.set_active_camera)

    def _on_fleet_state(self, raw):
        # 从 fleet/state 动态发现机器人，填充相机下拉
        import json
        try:
            agvs = json.loads(raw).get('agvs', [])
        except (ValueError, TypeError):
            return
        self._camera.set_robots([a.get('ns') for a in agvs if a.get('ns')])

    def closeEvent(self, ev):
        self._bridge.shutdown()
        super().closeEvent(ev)
```

- [ ] **Step 2: 改写 app.py（接通 rclpy + bridge + 窗口）**

```python
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
```

- [ ] **Step 3: 写 launch/qt_panel.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_ns', default_value='agv1',
                              description='哪台车的 /map 用于底图显示'),
        Node(
            package='amr_qt_panel',
            executable='amr_qt_panel',
            name='amr_qt_panel',
            output='screen',
            parameters=[{'map_ns': LaunchConfiguration('map_ns')}],
        ),
    ])
```

- [ ] **Step 4: 构建 + 冒烟（无头）**

Run:
```bash
cd /home/li/GazeboLib && colcon build --packages-select amr_qt_panel && source install/setup.bash
QT_QPA_PLATFORM=offscreen timeout 5 ros2 run amr_qt_panel amr_qt_panel; echo "exit=$?"
```
Expected: 进程启动无 Python 异常（`exit=124` 即被 timeout 正常杀掉）。

- [ ] **Step 5: 集成验收（有显示器时手动）**

按「仿真启停协议」起 `amr_vision_fleet`，再 `ros2 launch amr_qt_panel qt_panel.launch.py`，确认相机大画面出图、YOLO 框叠加、切车生效，视频明显比 web 稳。**不要 `kill -9` launch。**

- [ ] **Step 6: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/app.py src/amr_qt_panel/amr_qt_panel/widgets/main_window.py src/amr_qt_panel/launch/qt_panel.launch.py
git commit -m "feat(amr_qt_panel): wire RosBridge + CameraView + launch (phase 1: stable video)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## 阶段二：地图完整复刻

### Task 6: `coords.py` —— 世界↔像素变换（纯逻辑，TDD）

**Files:**
- Create: `src/amr_qt_panel/amr_qt_panel/model/coords.py`
- Test: `src/amr_qt_panel/test/test_coords.py`

**Interfaces:**
- Produces: `Transform(size=600, view=7.6)`，属性 `zoom,pan_x,pan_y,view,locked`；方法 `to_px(wx,wy)->(px,py)`、`to_len(m)->float`、`from_px(px,py)->(wx,wy)`、`calibrate_from_map(origin_x,origin_y,width,height,resolution)`。

- [ ] **Step 1: 写失败测试**

```python
from amr_qt_panel.model.coords import Transform


def test_round_trip():
    t = Transform(size=600, view=7.6)
    for wx, wy in [(0, 0), (3.5, -2.1), (-7, 7), (6.6, -6.6)]:
        px, py = t.to_px(wx, wy)
        rx, ry = t.from_px(px, py)
        assert abs(rx - wx) < 1e-6 and abs(ry - wy) < 1e-6


def test_round_trip_zoom_pan():
    t = Transform()
    t.zoom, t.pan_x, t.pan_y = 1.7, 40.0, -25.0
    px, py = t.to_px(2.0, -1.0)
    rx, ry = t.from_px(px, py)
    assert abs(rx - 2.0) < 1e-6 and abs(ry + 1.0) < 1e-6


def test_calibrate_from_map():
    t = Transform()
    t.calibrate_from_map(-10.0, -10.0, 200, 200, 0.1)   # extent ±10
    assert abs(t.view - 10.0 * 1.04) < 1e-6
    assert t.locked is True


def test_calibrate_respects_lock():
    t = Transform()
    t.locked = True
    t.calibrate_from_map(-10.0, -10.0, 200, 200, 0.1)
    assert t.view == 7.6
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd src/amr_qt_panel && python -m pytest test/test_coords.py -v`
Expected: FAIL（`ModuleNotFoundError`）。

- [ ] **Step 3: 实现 coords.py（移植 app.js:16,125-137,213-215）**

```python
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
```

- [ ] **Step 4: 跑测试确认通过**

Run: `cd src/amr_qt_panel && python -m pytest test/test_coords.py -v`
Expected: 4 passed。

- [ ] **Step 5: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/model/coords.py src/amr_qt_panel/test/test_coords.py
git commit -m "feat(amr_qt_panel): add world<->pixel Transform (ported from app.js)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 7: `model/fleet_state.py` —— 解析 /fleet/state（纯逻辑，TDD）

**Files:**
- Create: `src/amr_qt_panel/amr_qt_panel/model/fleet_state.py`
- Test: `src/amr_qt_panel/test/test_fleet_state.py`

**Interfaces:**
- Produces: `FleetState.from_json(str) -> FleetState`，含字段 `stamp, world_name, agvs:list[Agv], zones:dict[str,Zone], charger_zones:list, tasks:list[Task], anomalies:list[Anomaly], queued_tasks:int, idle_agvs:list, segment_owner, segment_occupants, segment_queue, corridor_segments, wait_points, collisions`。dataclass：`Agv(ns,x,y,yaw,battery,state,task,carrying,on_charger,nav_ready,stuck,home_x,home_y,semantic,semantic_stop)`、`Zone(name,cx,cy,sx,sy,gx,gy)`、`Task(id,pickup,dropoff,status,agv,agv_state)`、`Anomaly(level,ns,type,msg)`。

- [ ] **Step 1: 写失败测试**

```python
import json
from amr_qt_panel.model.fleet_state import FleetState

SAMPLE = json.dumps({
    "stamp": 1.0, "world_name": "warehouse",
    "agvs": [{"ns": "agv1", "x": 1.2, "y": -3.4, "yaw": 0.5, "battery": 0.8,
              "state": "TO_PICKUP", "task": "T1", "carrying": False,
              "on_charger": False, "nav_ready": True, "stuck": False,
              "home_x": 0.0, "home_y": 0.0, "semantic": "person",
              "semantic_stop": True}],
    "zones": {"pickup_zone_A": {"cx": -12.0, "cy": -6.0, "sx": 2.0, "sy": 2.0,
                                "gx": -12.0, "gy": -4.0}},
    "charger_zones": ["charger_1"],
    "tasks": [{"id": "T1", "pickup": "pickup_zone_A", "dropoff": "dropoff_zone_B",
               "status": "assigned", "agv": "agv1", "agv_state": "TO_PICKUP"}],
    "anomalies": [{"level": "warn", "ns": "agv1", "type": "行人停车", "msg": "..."}],
    "queued_tasks": 2, "idle_agvs": [],
    "corridor_segments": {"corridor_east": {"x_min": 15.0, "x_max": 20.0,
                                            "y_min": -13.0, "y_max": 13.0}},
    "segment_owner": {"corridor_east": "agv1"},
})


def test_parse_full():
    fs = FleetState.from_json(SAMPLE)
    assert fs.world_name == "warehouse"
    assert len(fs.agvs) == 1
    a = fs.agvs[0]
    assert a.ns == "agv1" and a.semantic == "person" and a.semantic_stop is True
    assert fs.zones["pickup_zone_A"].gx == -12.0
    assert fs.tasks[0].status == "assigned" and fs.tasks[0].agv == "agv1"
    assert fs.anomalies[0].level == "warn"
    assert fs.queued_tasks == 2
    assert fs.segment_owner["corridor_east"] == "agv1"


def test_parse_empty_tolerant():
    fs = FleetState.from_json("{}")
    assert fs.agvs == [] and fs.zones == {} and fs.queued_tasks == 0


def test_agv_missing_pose():
    fs = FleetState.from_json(json.dumps({"agvs": [{"ns": "agv2"}]}))
    a = fs.agvs[0]
    assert a.ns == "agv2" and a.x is None and a.battery == 0.0
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd src/amr_qt_panel && python -m pytest test/test_fleet_state.py -v`
Expected: FAIL（`ModuleNotFoundError`）。

- [ ] **Step 3: 实现 fleet_state.py**

```python
"""解析 /fleet/state JSON（schema 见 fleet_manager_ai.publish_fleet_state）。"""
import json
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Agv:
    ns: str
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: float = 0.0
    battery: float = 0.0
    state: str = ''
    task: Optional[str] = None
    carrying: bool = False
    on_charger: bool = False
    nav_ready: bool = False
    stuck: bool = False
    home_x: Optional[float] = None
    home_y: Optional[float] = None
    semantic: str = 'clear'
    semantic_stop: bool = False


@dataclass
class Zone:
    name: str
    cx: float
    cy: float
    sx: float
    sy: float
    gx: float
    gy: float


@dataclass
class Task:
    id: str
    pickup: Optional[str]
    dropoff: Optional[str]
    status: str
    agv: Optional[str]
    agv_state: Optional[str]


@dataclass
class Anomaly:
    level: str
    ns: str
    type: str
    msg: str


@dataclass
class FleetState:
    stamp: float = 0.0
    world_name: str = ''
    agvs: list = field(default_factory=list)
    zones: dict = field(default_factory=dict)
    charger_zones: list = field(default_factory=list)
    tasks: list = field(default_factory=list)
    anomalies: list = field(default_factory=list)
    queued_tasks: int = 0
    idle_agvs: list = field(default_factory=list)
    segment_owner: dict = field(default_factory=dict)
    segment_occupants: dict = field(default_factory=dict)
    segment_queue: dict = field(default_factory=dict)
    corridor_segments: dict = field(default_factory=dict)
    wait_points: dict = field(default_factory=dict)
    collisions: list = field(default_factory=list)

    @classmethod
    def from_json(cls, raw: str) -> 'FleetState':
        d = json.loads(raw)
        agvs = [Agv(
            ns=a.get('ns', ''), x=a.get('x'), y=a.get('y'),
            yaw=a.get('yaw', 0.0), battery=a.get('battery', 0.0),
            state=a.get('state', ''), task=a.get('task'),
            carrying=a.get('carrying', False), on_charger=a.get('on_charger', False),
            nav_ready=a.get('nav_ready', False), stuck=a.get('stuck', False),
            home_x=a.get('home_x'), home_y=a.get('home_y'),
            semantic=a.get('semantic', 'clear'),
            semantic_stop=a.get('semantic_stop', False),
        ) for a in d.get('agvs', [])]

        zones = {n: Zone(
            name=n, cx=z['cx'], cy=z['cy'],
            sx=z.get('sx', 1.0), sy=z.get('sy', 1.0),
            gx=z.get('gx', z['cx']), gy=z.get('gy', z['cy']),
        ) for n, z in d.get('zones', {}).items()}

        tasks = [Task(
            id=str(t.get('id', '?')), pickup=t.get('pickup'),
            dropoff=t.get('dropoff'), status=t.get('status', ''),
            agv=t.get('agv'), agv_state=t.get('agv_state'),
        ) for t in d.get('tasks', [])]

        anomalies = [Anomaly(
            level=a.get('level', 'info'), ns=a.get('ns', ''),
            type=a.get('type', ''), msg=a.get('msg', ''),
        ) for a in d.get('anomalies', [])]

        return cls(
            stamp=d.get('stamp', 0.0), world_name=d.get('world_name', ''),
            agvs=agvs, zones=zones, charger_zones=d.get('charger_zones', []),
            tasks=tasks, anomalies=anomalies,
            queued_tasks=d.get('queued_tasks', 0), idle_agvs=d.get('idle_agvs', []),
            segment_owner=d.get('segment_owner', {}),
            segment_occupants=d.get('segment_occupants', {}),
            segment_queue=d.get('segment_queue', {}),
            corridor_segments=d.get('corridor_segments', {}),
            wait_points=d.get('wait_points', {}),
            collisions=d.get('collisions', []),
        )
```

- [ ] **Step 4: 跑测试确认通过**

Run: `cd src/amr_qt_panel && python -m pytest test/test_fleet_state.py -v`
Expected: 3 passed。

- [ ] **Step 5: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/model/fleet_state.py src/amr_qt_panel/test/test_fleet_state.py
git commit -m "feat(amr_qt_panel): add FleetState JSON model

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 8: `map_view.py` —— QPainter 地图（栅格/静态/路径/AGV，缩放，点击导航）

**Files:**
- Create: `src/amr_qt_panel/amr_qt_panel/widgets/map_view.py`
- Test: `src/amr_qt_panel/test/test_map_view.py`

**Interfaces:**
- Consumes: `FleetState`（agvs/zones/corridor_segments/wait_points）、`OccupancyGrid`、`path_changed(ns, [(x,y)])`、`coords.Transform`。
- Produces: `MapView(QWidget)`；信号 `map_clicked(float, float)`（世界坐标，供上层在已选车时调用 `publish_goal`）；槽 `set_fleet_state(FleetState)`、`set_map(ns, OccupancyGrid)`、`set_path(ns, list)`、`set_selected_robot(ns)`。

**移植参考（amr_web/app.js）**：占用栅格离屏位图 `onMapMsg`(177-216)+`drawMap`(225-235)；静态图元 `drawStatic`(238-346)（货架/取货/卸货/充电色、接近点 gx/gy、走廊段、等待点、标签）；路径 `drawPaths`(355-367)；AGV 三角与电量色 `batteryColor`(348-)。颜色/图例以 `index.html` legend（31-42 行）为准。

- [ ] **Step 1: 写 offscreen 冒烟测试**

```python
import json
import numpy as np
from types import SimpleNamespace
from amr_qt_panel.model.fleet_state import FleetState
from amr_qt_panel.widgets.map_view import MapView


def _grid():
    info = SimpleNamespace(resolution=0.1, width=20, height=20,
                           origin=SimpleNamespace(
                               position=SimpleNamespace(x=-1.0, y=-1.0, z=0.0)))
    data = [0] * (20 * 20)
    return SimpleNamespace(info=info, data=data)


def test_render_smoke(qapp):
    v = MapView()
    v.resize(600, 600)
    fs = FleetState.from_json(json.dumps({
        "agvs": [{"ns": "agv1", "x": 0.0, "y": 0.0, "yaw": 0.0, "battery": 0.7,
                  "state": "IDLE"}],
        "zones": {"pickup_zone_A": {"cx": -2.0, "cy": -1.0, "sx": 1.0, "sy": 1.0}},
    }))
    v.set_map('agv1', _grid())
    v.set_fleet_state(fs)
    v.set_path('agv1', [(0.0, 0.0), (1.0, 1.0)])
    pm = v.grab()                       # 不应抛异常
    assert pm.width() > 0


def test_click_emits_world(qapp):
    v = MapView()
    v.resize(600, 600)
    got = []
    v.map_clicked.connect(lambda x, y: got.append((x, y)))
    v._emit_click_at(300, 300)          # 测试钩子：用画布中心像素
    assert got and len(got[0]) == 2
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_map_view.py -v`
Expected: FAIL（`ModuleNotFoundError`）。

- [ ] **Step 3: 实现 map_view.py**

骨架与关键逻辑如下（静态图元的具体着色按上面「移植参考」逐项补全；以下代码已可通过冒烟测试）：

```python
"""MapView：QPainter 重画占用栅格 + 静态图元 + Nav2 路径 + AGV；滚轮缩放、点击导航。"""
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

    # ---- 槽 ----
    def set_fleet_state(self, fs):
        self._fs = fs
        self.update()

    def set_map(self, ns, grid):
        info = grid.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        self._t.calibrate_from_map(ox, oy, w, h, res)
        # 占用栅格 -> QImage（左下角=origin，需上下翻转）
        img = QImage(w, h, QImage.Format_RGB888)
        for row in range(h):
            for col in range(w):
                v = grid.data[row * w + col]
                if v < 0:
                    c = 130           # 未知=灰
                elif v >= 65:
                    c = 30            # 占据=深
                else:
                    c = 235           # 空闲=浅
                img.setPixel(col, h - 1 - row, (c << 16) | (c << 8) | c)
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
        for i, (ns, pts) in enumerate(self._paths.items()):
            if not pts:
                continue
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
            color = QColor(AGV_COLORS[i % len(AGV_COLORS)])
            import math
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
    def wheelEvent(self, ev):
        factor = 1.1 if ev.angleDelta().y() > 0 else 1 / 1.1
        self._t.zoom *= factor
        self.update()

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._emit_click_at(ev.x(), ev.y())

    def _emit_click_at(self, px, py):
        wx, wy = self._t.from_px(px, py)
        self.map_clicked.emit(float(wx), float(wy))
```

- [ ] **Step 4: 跑测试确认通过**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_map_view.py -v`
Expected: 2 passed。

- [ ] **Step 5: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/widgets/map_view.py src/amr_qt_panel/test/test_map_view.py
git commit -m "feat(amr_qt_panel): add MapView (grid/zones/paths/AGVs, zoom, click-to-nav)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 9: 把地图接入主窗口（左地图 + 右相机）

**Files:**
- Modify: `src/amr_qt_panel/amr_qt_panel/widgets/main_window.py`

**Interfaces:**
- Consumes: `MapView`、`RosBridge.path_changed/map_changed/fleet_state_changed`、`FleetState.from_json`。

- [ ] **Step 1: 改 main_window.py——左 MapView + 右 CameraView，集中解析 fleet_state**

```python
"""MainWindow：左地图 + 右侧栏（阶段二：地图+相机）。"""
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout

from amr_qt_panel.model.fleet_state import FleetState
from amr_qt_panel.widgets.camera_view import CameraView
from amr_qt_panel.widgets.map_view import MapView


class MainWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self._bridge = bridge
        self.setWindowTitle("AGV 仓库调度中心 (QT)")
        self.resize(1400, 860)
        self._selected = None

        central = QWidget()
        root = QHBoxLayout(central)
        self._map = MapView()
        root.addWidget(self._map, 3)

        side = QVBoxLayout()
        self._camera = CameraView()
        side.addWidget(self._camera, 1)
        root.addLayout(side, 2)
        self.setCentralWidget(central)

        # 接线
        bridge.image_changed.connect(self._camera.set_image)
        bridge.detections_changed.connect(self._camera.set_detections)
        bridge.map_changed.connect(self._map.set_map)
        bridge.path_changed.connect(self._map.set_path)
        bridge.fleet_state_changed.connect(self._on_fleet_state_raw)
        self._camera.camera_selected.connect(bridge.set_active_camera)
        self._map.map_clicked.connect(self._on_map_clicked)

    def _on_fleet_state_raw(self, raw):
        try:
            fs = FleetState.from_json(raw)
        except (ValueError, TypeError):
            return
        self._map.set_fleet_state(fs)
        robots = [a.ns for a in fs.agvs if a.ns]
        self._camera.set_robots(robots)
        for ns in robots:
            self._bridge.ensure_path_sub(ns)

    def _on_map_clicked(self, wx, wy):
        if self._selected:
            self._bridge.publish_goal(self._selected, wx, wy)

    def closeEvent(self, ev):
        self._bridge.shutdown()
        super().closeEvent(ev)
```

- [ ] **Step 2: 构建 + 无头冒烟**

Run:
```bash
cd /home/li/GazeboLib && colcon build --packages-select amr_qt_panel && source install/setup.bash
QT_QPA_PLATFORM=offscreen timeout 5 ros2 run amr_qt_panel amr_qt_panel; echo "exit=$?"
```
Expected: 无 Python 异常（`exit=124`）。

- [ ] **Step 3: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/widgets/main_window.py
git commit -m "feat(amr_qt_panel): integrate MapView into main window (phase 2)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## 阶段三：各表 / 下发任务 / 日志（全量对齐 web）

### Task 10: 表格面板（任务分配 / 车队状态 / 路权 / 系统异常）

**Files:**
- Create: `src/amr_qt_panel/amr_qt_panel/widgets/panels.py`
- Test: `src/amr_qt_panel/test/test_panels.py`

**Interfaces:**
- Consumes: `FleetState`。
- Produces: `TaskTable(QWidget)`、`FleetTable(QWidget)`、`RowTable(QWidget)`、`AnomalyList(QWidget)`，各有 `update_state(fs)`；`FleetTable` 信号 `robot_selected(str)`（点行选车）。

- [ ] **Step 1: 写 offscreen 测试**

```python
import json
from amr_qt_panel.model.fleet_state import FleetState
from amr_qt_panel.widgets.panels import TaskTable, FleetTable, RowTable, AnomalyList

FS = FleetState.from_json(json.dumps({
    "agvs": [{"ns": "agv1", "x": 1.0, "y": 2.0, "battery": 0.55, "state": "TO_PICKUP",
              "carrying": True, "home_x": 0.0, "home_y": 0.0}],
    "tasks": [{"id": "T1", "pickup": "A", "dropoff": "B", "status": "assigned",
               "agv": "agv1", "agv_state": "TO_PICKUP"}],
    "anomalies": [{"level": "error", "ns": "agv1", "type": "碰撞", "msg": "危险"}],
    "corridor_segments": {"corridor_east": {"x_min": 15.0, "x_max": 20.0,
                                            "y_min": -13.0, "y_max": 13.0}},
    "segment_owner": {"corridor_east": "agv1"},
    "segment_queue": {"corridor_east": ["agv2"]},
}))


def test_task_table(qapp):
    t = TaskTable(); t.update_state(FS)
    assert t.rowCount() == 1


def test_fleet_table_select(qapp):
    t = FleetTable(); t.update_state(FS)
    got = []
    t.robot_selected.connect(got.append)
    t.selectRow(0)
    assert got and got[-1] == 'agv1'


def test_row_table(qapp):
    t = RowTable(); t.update_state(FS)
    assert t.rowCount() == 1


def test_anomaly_list(qapp):
    a = AnomalyList(); a.update_state(FS)
    assert a.count() == 1
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_panels.py -v`
Expected: FAIL（`ModuleNotFoundError`）。

- [ ] **Step 3: 实现 panels.py**

```python
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

    def update_state(self, fs):
        rows = [(t.id, t.agv or '-', t.status,
                 f"{t.pickup or '?'} → {t.dropoff or '?'}") for t in fs.tasks]
        self._fill(rows)


class FleetTable(_Table):
    robot_selected = pyqtSignal(str)

    def __init__(self):
        super().__init__(["AGV", "状态", "电量", "载货", "位置(x,y)", "待命点"])
        self._rows = []
        self.itemSelectionChanged.connect(self._on_sel)

    def update_state(self, fs):
        self._rows = [a.ns for a in fs.agvs]
        rows = []
        for a in fs.agvs:
            pos = "-" if a.x is None else f"({a.x:.1f}, {a.y:.1f})"
            home = "-" if a.home_x is None else f"({a.home_x:.1f}, {a.home_y:.1f})"
            rows.append((a.ns, a.state, f"{a.battery*100:.0f}%",
                         "是" if a.carrying else "否", pos, home))
        self._fill(rows)

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
```

- [ ] **Step 4: 跑测试确认通过**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_panels.py -v`
Expected: 4 passed。

- [ ] **Step 5: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/widgets/panels.py src/amr_qt_panel/test/test_panels.py
git commit -m "feat(amr_qt_panel): add task/fleet/row/anomaly panels

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 11: 下发任务表单 + /rosout 日志

**Files:**
- Create: `src/amr_qt_panel/amr_qt_panel/widgets/task_form.py`
- Create: `src/amr_qt_panel/amr_qt_panel/widgets/log_list.py`
- Test: `src/amr_qt_panel/test/test_task_form.py`

**Interfaces:**
- Produces:
  - `TaskForm(QWidget)`：槽 `update_zones(fs)`（用非充电区填充取货/卸货下拉）；信号 `dispatch(str, str)`（pickup, dropoff）。
  - `LogList(QWidget)`：槽 `append_log(msg)`（rcl_interfaces/Log：`msg.level,msg.name,msg.msg`）；INFO 复选框 + 清空按钮。Log level：DEBUG=10 INFO=20 WARN=30 ERROR=40 FATAL=50。

- [ ] **Step 1: 写 offscreen 测试**

```python
import json
from types import SimpleNamespace
from amr_qt_panel.model.fleet_state import FleetState
from amr_qt_panel.widgets.task_form import TaskForm
from amr_qt_panel.widgets.log_list import LogList

FS = FleetState.from_json(json.dumps({
    "zones": {"pickup_zone_A": {"cx": -12.0, "cy": -6.0},
              "dropoff_zone_B": {"cx": 12.0, "cy": 10.0},
              "charger_1": {"cx": -20.0, "cy": -10.0}},
    "charger_zones": ["charger_1"], "queued_tasks": 3,
}))


def test_task_form_zones_exclude_charger(qapp):
    f = TaskForm(); f.update_zones(FS)
    items = [f._pickup.itemText(i) for i in range(f._pickup.count())]
    assert "pickup_zone_A" in items and "charger_1" not in items


def test_task_form_dispatch(qapp):
    f = TaskForm(); f.update_zones(FS)
    got = []
    f.dispatch.connect(lambda p, d: got.append((p, d)))
    f._pickup.setCurrentText("pickup_zone_A")
    f._dropoff.setCurrentText("dropoff_zone_B")
    f._btn.click()
    assert got == [("pickup_zone_A", "dropoff_zone_B")]


def test_log_info_toggle(qapp):
    lg = LogList()
    info = SimpleNamespace(level=20, name="nav", msg="hello")
    err = SimpleNamespace(level=40, name="ctrl", msg="boom")
    lg.append_log(info)     # 默认不显示 INFO
    lg.append_log(err)
    assert lg._list.count() == 1
    lg._show_info.setChecked(True)
    lg.append_log(info)
    assert lg._list.count() == 2
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_task_form.py -v`
Expected: FAIL（`ModuleNotFoundError`）。

- [ ] **Step 3: 实现 task_form.py**

```python
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
```

- [ ] **Step 4: 实现 log_list.py**

```python
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
```

- [ ] **Step 5: 跑测试确认通过**

Run: `cd src/amr_qt_panel && QT_QPA_PLATFORM=offscreen python -m pytest test/test_task_form.py -v`
Expected: 3 passed。

- [ ] **Step 6: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/widgets/task_form.py src/amr_qt_panel/amr_qt_panel/widgets/log_list.py src/amr_qt_panel/test/test_task_form.py
git commit -m "feat(amr_qt_panel): add task dispatch form and rosout log list

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 12: 全量总装（右侧栏滚动容器 + 全部接线 + 验收）

**Files:**
- Modify: `src/amr_qt_panel/amr_qt_panel/widgets/main_window.py`

**Interfaces:**
- Consumes: 阶段三全部 widget + `RosBridge.log_received` + `publish_add_task`。

- [ ] **Step 1: 改 main_window.py——右侧 QScrollArea 装全部面板并接线**

```python
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
```

- [ ] **Step 2: 全量构建 + 全测试 + 无头冒烟**

Run:
```bash
cd /home/li/GazeboLib && colcon build --packages-select amr_qt_panel && source install/setup.bash
QT_QPA_PLATFORM=offscreen python -m pytest src/amr_qt_panel/test -v
QT_QPA_PLATFORM=offscreen timeout 5 ros2 run amr_qt_panel amr_qt_panel; echo "exit=$?"
```
Expected: 全部 test passed；进程无异常（`exit=124`）。

- [ ] **Step 3: 集成验收（有显示器，连真实仿真）**

按「仿真启停协议」起 `amr_vision_fleet`，再起本面板，逐项核对：地图（栅格/区/走廊/路径/AGV朝向/缩放/点击导航）、相机视频流畅+YOLO框+切车、任务分配/车队/路权/异常各表有数、下发任务进队、`/rosout` 日志滚动。与 web 面板并排对照一致。**结束按协议优雅关闭，不要 `kill -9`。**

- [ ] **Step 4: Commit**

```bash
git add src/amr_qt_panel/amr_qt_panel/widgets/main_window.py
git commit -m "feat(amr_qt_panel): full web-panel parity assembly (phase 3)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## 自检对照（spec → task 覆盖）

| spec 要求 | 对应 Task |
|---|---|
| 新 ament_python 包 / 入口 / launch | Task 1, 5 |
| 绕开 web_video_server，直订图像 | Task 3, 4 |
| Image→QImage（无 cv2） | Task 2 |
| YOLO 叠框 + 切车 | Task 4 |
| RosBridge 线程+信号+QoS+发布 | Task 3 |
| 世界↔像素变换（含定标/缩放） | Task 6 |
| /fleet/state 模型 | Task 7 |
| 地图完整复刻（栅格/静态/路径/AGV/点击导航） | Task 8, 9 |
| 任务分配/车队/路权/异常表 | Task 10 |
| 下发任务 / /rosout 日志 | Task 11 |
| 全量总装 + 验收 | Task 12 |
| 纯逻辑无头单测 | Task 2,3,6,7 + widget 冒烟 4,8,10,11 |

> 说明：Task 8 的静态图元着色（货架/取货/卸货分类色、接近点、等待点、图例细节）需对照 `app.js` `drawStatic`(238-346) 与 `index.html` legend 逐项补全，代码骨架已给出可运行版本，视觉细节在集成验收阶段与 web 对照微调。
