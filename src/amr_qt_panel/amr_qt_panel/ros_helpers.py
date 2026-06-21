"""纯函数 helper：与 ROS/Qt 无关，可独立单测（不需 source ROS）。"""
import json
import math


def add_task_payload(pickup: str, dropoff: str) -> str:
    return json.dumps({'pickup': pickup, 'dropoff': dropoff})


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def camera_topics(ns: str):
    return (f'/{ns}/camera/image_raw', f'/{ns}/camera/detections')
