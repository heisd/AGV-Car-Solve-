"""RosBridge：唯一 ROS 边界。后台线程跑执行器，回调只 emit Qt 信号。"""
import json
import threading

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
from amr_qt_panel.ros_helpers import add_task_payload, camera_topics, yaw_to_quat


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
        self._node.declare_parameter('map_ns', map_ns)
        map_ns = self._node.get_parameter('map_ns').get_parameter_value().string_value
        self._node.get_logger().info(f"amr_qt_panel map_ns={map_ns}")
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
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
        except Exception as e:
            self._node.get_logger().warn(f"executor spin ended: {e}")

    def shutdown(self):
        self._exec.shutdown()
        self._thread.join(timeout=2.0)
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
