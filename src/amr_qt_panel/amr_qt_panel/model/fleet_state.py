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
