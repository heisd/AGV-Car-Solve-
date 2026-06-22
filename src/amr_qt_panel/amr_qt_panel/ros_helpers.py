"""纯函数 helper：与 ROS/Qt 无关，可独立单测（不需 source ROS）。"""
import json
import math


def add_task_payload(pickup: str, dropoff: str) -> str:
    return json.dumps({'pickup': pickup, 'dropoff': dropoff})


def charge_payload(agv: str, charger: str) -> str:
    """手动充电指令：把指定车派往指定充电桩（fleet_manager 按 type 分支处理）。"""
    return json.dumps({'type': 'charge', 'agv': agv, 'charger': charger})


def cancel_task_payload(task_id: str) -> str:
    """按任务 id 取消（排队任务或执行中任务）。"""
    return json.dumps({'id': task_id})


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def camera_topics(ns: str):
    return (f'/{ns}/camera/image_raw', f'/{ns}/camera/detections')
