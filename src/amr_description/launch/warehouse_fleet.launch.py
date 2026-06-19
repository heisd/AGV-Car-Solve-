# warehouse_fleet.launch.py
#
# 一键拉起"多车 + 调度中心 + Web 面板"系统（免建图，真值跟随器驱动）：
#   - Gazebo Classic + warehouse.world (20 货架 + 取货点 + 充电站)
#   - 3 台 AGV：robot_state_publisher / spawn / odom_sim_filter / battery_sim /
#               charger_dock_monitor / obstacle_detection / ground_truth_waypoint_follower
#   - 调度中心 fleet_manager（读 warehouse_tasks.yaml，发 /<ns>/goal_pose，开放 /fleet/* 给 Web）
#   - Web 面板层 web_panel.launch.py（rosbridge + 静态网页），use_web:=false 可关闭
#
# 用法：
#   ros2 launch amr_description warehouse_fleet.launch.py
#   浏览器打开 http://localhost:8080/
#
# 参数：world / use_web / num_agvs 暂固定 3 台（agv1..agv3）。

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 三台车的命名空间与初始位姿（放在四周走廊，便于直线驶向任务区）
AGVS = [
    ('agv1', 5.5, 0.0),
    ('agv2', -5.5, 0.0),
    ('agv3', 0.0, -6.0),
]
CHARGER_XY = [-5.5, -5.5]   # 充电站坐标（charger_dock_monitor 用）


def generate_launch_description():
    pkg_amr = get_package_share_directory('amr_description')
    pkg_web = get_package_share_directory('amr_web')

    default_world = os.path.join(pkg_amr, 'worlds', 'warehouse.world')
    tasks_file = os.path.join(pkg_amr, 'yaml', 'warehouse_tasks.yaml')
    xacro_path = os.path.join(pkg_amr, 'urdf', 'amr.urdf.xacro')

    world = LaunchConfiguration('world')
    use_web = LaunchConfiguration('use_web')

    actions = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world 绝对路径。'),
        DeclareLaunchArgument('use_web', default_value='true',
                              description='是否同时启动 Web 面板层 (rosbridge + 网页)。'),
        LogInfo(msg='[warehouse_fleet] 启动 3 台 AGV + 调度中心；Web 面板: http://localhost:8080/'),
    ]

    # Gazebo Classic + warehouse 世界
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    ))

    namespaces = []
    for i, (ns, x, y) in enumerate(AGVS):
        namespaces.append(ns)
        # 启动时即用 xacro 展开为临时 URDF：robot_state_publisher 与 spawn 用同一份。
        # spawn 用 -file（不依赖话题），避免并发启动下 spawn_entity 拿不到 latched
        # /<ns>/robot_description 而死等 xml、最终不 spawn 的问题。
        urdf_xml = subprocess.check_output(['xacro', xacro_path, f'namespace:={ns}']).decode()
        urdf_file = os.path.join(tempfile.gettempdir(), f'warehouse_{ns}.urdf')
        with open(urdf_file, 'w') as fh:
            fh.write(urdf_xml)

        actions += [
            Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                name='robot_state_publisher', namespace=ns,
                parameters=[{'robot_description': urdf_xml, 'use_sim_time': True}],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen',
            ),
            # 延时错峰 spawn：等 Gazebo 的 /spawn_entity 服务就绪 (~7.5s) 后再从文件插入模型。
            TimerAction(period=5.0 + 2.0 * i, actions=[
                Node(
                    package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_{ns}',
                    arguments=['-entity', ns, '-file', urdf_file,
                               '-x', str(x), '-y', str(y), '-z', '0.1'],
                    output='screen',
                ),
            ]),
            Node(
                package='amr_description', executable='odom_sim_filter', name='odom_sim_filter',
                namespace=ns,
                parameters=[{'use_sim_time': True, 'robot_namespace': ns}],
                output='screen', emulate_tty=True,
            ),
            Node(
                package='amr_description', executable='battery_sim', name='battery_sim',
                namespace=ns,
                parameters=[{'use_sim_time': True, 'robot_namespace': ns,
                             'charger_contact_topic': f'/{ns}/on_charger'}],
                output='screen', emulate_tty=True,
            ),
            Node(
                package='amr_description', executable='charger_dock_monitor', name='charger_dock_monitor',
                namespace=ns,
                parameters=[{'use_sim_time': True, 'robot_namespace': ns,
                             'odom_topic': f'/{ns}/ground_truth',
                             'contact_topic': f'/{ns}/on_charger',
                             'charger_centers_xy': CHARGER_XY}],
                output='screen', emulate_tty=True,
            ),
            Node(
                package='amr_description', executable='obstacle_detection_node', name='obstacle_detection',
                namespace=ns,
                parameters=[{'use_sim_time': True, 'robot_namespace': ns}],
                output='screen', emulate_tty=True,
            ),
            Node(
                package='amr_description', executable='ground_truth_waypoint_follower.py',
                name='ground_truth_waypoint_follower', namespace=ns,
                parameters=[{'use_sim_time': True, 'robot_namespace': ns,
                             'goal_reach_dist': 0.25, 'max_linear': 0.5}],
                output='screen', emulate_tty=True,
            ),
        ]

    # 调度中心
    actions.append(Node(
        package='amr_description', executable='fleet_manager', name='fleet_manager',
        parameters=[{
            'use_sim_time': True,
            'robot_namespaces': namespaces,
            'tasks_file': tasks_file,
            'battery_topic_type': 'battery_state',
            'charger_zone_names': ['charger_1'],
            'battery_low_threshold': 0.20,
            'battery_resume_threshold': 0.60,
            'goal_reach_dist': 0.30,
            'require_nav_ready': False,
        }],
        output='screen', emulate_tty=True,
    ))

    # Web 面板层（rosbridge + 静态网页）
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_web, 'launch', 'web_panel.launch.py')),
        condition=IfCondition(use_web),
    ))

    return LaunchDescription(actions)
