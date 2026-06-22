import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """QT 桌面面板 + (可选) 一并拉起 amr_vision_fleet（不含网页端）。

    QT 面板直接订阅 ROS 2 图像话题，不需要 web_video_server / rosbridge /
    http 静态服务器，因此把 fleet 的 launch_web 关掉（launch_web:=false）。
    其余功能（Nav2 / 相机检测 / fleet_manager_ai / 可选 Gazebo）全部保留。
    """
    map_ns = LaunchConfiguration('map_ns')
    launch_fleet = LaunchConfiguration('launch_fleet')

    fleet_launch = os.path.join(
        get_package_share_directory('amr_vision'),
        'launch', 'amr_vision_fleet.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_ns', default_value='agv1',
            description='哪台车的 /map 用于面板底图显示。'),
        DeclareLaunchArgument(
            'launch_fleet', default_value='true',
            description='true=同时拉起 amr_vision_fleet（Nav2/相机/fleet_manager，'
                        '但不含网页端）；false=只起 QT 面板，连已在运行的 fleet。'),
        # ---- 以下参数透传给 amr_vision_fleet（仅 launch_fleet:=true 时生效）----
        DeclareLaunchArgument(
            'num_agvs', default_value='1',
            description='车队数量（透传给 amr_vision_fleet）。'),
        DeclareLaunchArgument(
            'launch_gazebo', default_value='false',
            description='true=独立模式自起 Gazebo 并 spawn 机器人；'
                        'false=假设 display_vision/spawn_agv 已在运行（透传）。'),
        DeclareLaunchArgument(
            'spawn_poses', default_value='0.0,0.0',
            description='分号分隔的 "x,y" 生成位姿，每车一个（透传给 amr_vision_fleet）。'),
        DeclareLaunchArgument(
            'model_path', default_value='yolov8n.pt',
            description='YOLO 权重路径，传给每个 camera_detection_node（透传）。'),

        # 拉起 amr_vision_fleet，但 launch_web:=false —— 关掉网页端
        # （rosbridge + http.server + web_video_server），其余功能保留。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fleet_launch),
            condition=IfCondition(launch_fleet),
            launch_arguments={
                'launch_web': 'false',
                'num_agvs': LaunchConfiguration('num_agvs'),
                'launch_gazebo': LaunchConfiguration('launch_gazebo'),
                'spawn_poses': LaunchConfiguration('spawn_poses'),
                'model_path': LaunchConfiguration('model_path'),
            }.items(),
        ),

        # QT 桌面面板节点
        Node(
            package='amr_qt_panel',
            executable='amr_qt_panel',
            name='amr_qt_panel',
            output='screen',
            parameters=[{'map_ns': map_ns}],
        ),
    ])