# warehouse_fleet_nav2.launch.py
#
# 多车 + 调度中心 + Web 面板，**Nav2 驱动**（真实路径规划 + 动态避障）。
#   - Gazebo Classic + warehouse.world
#   - 3 台 AGV：rsp + spawn(-file) + odom_sim_filter + battery_sim + charger_dock_monitor + tf_relay
#   - 每车一套命名空间化 Nav2 栈 (nav2_bringup) + nav2_goal_bridge(把 /<ns>/goal_pose 转 NavigateToPose 并给 AMCL 播种)
#   - 调度中心 fleet_manager（warehouse_tasks.yaml，发 /<ns>/goal_pose，开放 /fleet/* 给 Web）
#   - Web 面板层 (rosbridge + 网页)，use_web:=false 可关
#
# 地图：maps/warehouse_map.yaml（由仓库几何确定性生成，匹配 warehouse.world）
# 用法：ros2 launch amr_description warehouse_fleet_nav2.launch.py ；浏览器 http://localhost:8080/
#
# 说明：WSL2 上同时拉 3 套 Nav2 较重，Nav2 按车错峰启动 (12/24/36s)；spawn 错峰 (4/6/8s)。

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 命名空间 + 初始位姿（= Gazebo spawn 位姿 = AMCL 初始位姿，三者必须一致）
AGVS = [
    ('agv1', 5.5, 0.0),
    ('agv2', -5.5, 0.0),
    ('agv3', 0.0, -6.0),
]
CHARGER_XY = [-5.5, -5.5]


def launch_setup(context, *args, **kwargs):
    # Ensure ~/.gazebo/models is in GAZEBO_MODEL_PATH so Gazebo can find the models
    gazebo_model_path = os.path.expanduser('~/.gazebo/models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        if gazebo_model_path not in os.environ['GAZEBO_MODEL_PATH'].split(os.pathsep):
            os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    pkg_amr = get_package_share_directory('amr_description')
    pkg_web = get_package_share_directory('amr_web')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_world = os.path.join(pkg_amr, 'worlds', 'warehouse.world')
    world = context.launch_configurations.get('world', default_world)
    map_file = os.path.join(pkg_amr, 'maps', 'warehouse_map.yaml')
    tasks_file = os.path.join(pkg_amr, 'yaml', 'warehouse_tasks.yaml')
    xacro_path = os.path.join(pkg_amr, 'urdf', 'amr.urdf.xacro')
    params_template = os.path.join(pkg_amr, 'yaml', 'nav2_params_amr.yaml')
    bt_file = os.path.join(pkg_amr, 'behavior_trees', 'navigate_w_recovery.xml')

    use_web = context.launch_configurations.get('use_web', 'true').lower() == 'true'
    gui = context.launch_configurations.get('gui', 'true').lower() == 'true'
    num_agvs = max(1, min(len(AGVS), int(context.launch_configurations.get('num_agvs', str(len(AGVS))))))
    agvs = AGVS[:num_agvs]

    with open(params_template, 'r') as f:
        params_tmpl = f.read()

    nodes = [LogInfo(msg='[warehouse_fleet_nav2] Gazebo + 3×AGV(Nav2) + 调度中心 + Web(http://localhost:8080/)')]

    # Gazebo Classic + warehouse 世界
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': 'true' if gui else 'false'}.items(),
    ))

    ns_list = []
    for i, (ns, x, y) in enumerate(agvs):
        ns_list.append(ns)

        # 启动时展开 URDF；spawn 用 -file（不依赖 latched 话题）
        urdf_xml = subprocess.check_output(['xacro', xacro_path, f'namespace:={ns}']).decode()
        urdf_file = os.path.join(tempfile.gettempdir(), f'warehouse_{ns}.urdf')
        with open(urdf_file, 'w') as fh:
            fh.write(urdf_xml)

        # spawn 节点（命名变量，供事件驱动用）。负载高/世界大时 gazebo 的 /spawn_entity
        # 服务可能数十秒才就绪，故 -timeout 120 让其耐心等待，不会过早超时退出。
        spawn_node = Node(
            package='gazebo_ros', executable='spawn_entity.py', name=f'spawn_{ns}',
            arguments=['-entity', ns, '-file', urdf_file,
                       '-x', str(x), '-y', str(y), '-z', '0.1',
                       '-timeout', '120.0'],
            output='screen',
        )

        nodes += [
            Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                name='robot_state_publisher', namespace=ns,
                parameters=[{'robot_description': urdf_xml, 'use_sim_time': True}],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen',
            ),
            # 错峰 spawn（8/11/14s 等 gazebo 起来）
            TimerAction(period=8.0 + 3.0 * i, actions=[spawn_node]),
            Node(
                package='amr_description', executable='odom_sim_filter', name='odom_sim_filter',
                namespace=ns, parameters=[{'use_sim_time': True, 'robot_namespace': ns}],
                remappings=[('/tf', 'tf')], output='screen', emulate_tty=True,
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
            # tf_relay：把全局 /tf,/tf_static 转到 /<ns>/tf，供命名空间化 Nav2 读取
            Node(
                package='amr_description', executable='tf_relay', name='tf_relay',
                namespace=ns, parameters=[{'use_sim_time': True, 'robot_namespace': ns}],
                output='screen', emulate_tty=True,
            ),
        ]

        # 每车 Nav2 参数（替换 {NS}/{BT_FILE}）写临时文件
        params_content = params_tmpl.replace('{NS}', ns).replace('{BT_FILE}', bt_file)
        tmp = tempfile.NamedTemporaryFile(mode='w', suffix=f'_{ns}_nav2.yaml', delete=False)
        tmp.write(params_content)
        tmp.close()

        nav2_actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'namespace': ns,
                    'use_namespace': 'True',
                    'slam': 'False',
                    'map': map_file,
                    'use_sim_time': 'True',
                    'params_file': tmp.name,
                    'autostart': 'True',
                    # 关键降耗：每套 Nav2 合并进单个组件容器，启用进程内通信(IPC)，
                    # 进程数从 ~7/车 降到 1/车。错峰启动(12/24/36s)避免多容器 load_node 抢服务。
                    'use_composition': 'True',
                    'use_respawn': 'False',
                }.items(),
            ),
            Node(
                package='amr_description', executable='nav2_goal_bridge', name='nav2_goal_bridge',
                namespace=ns,
                parameters=[{'use_sim_time': True, 'robot_namespace': ns,
                             'initial_x': x, 'initial_y': y}],
                output='screen', emulate_tty=True,
            ),
        ]
        # 事件驱动：等本车 spawn 进程退出(成功生成)后再启动其 Nav2。
        # 这样 Nav2(AMCL/costmap) 一定在机器人存在、能出 scan 之后才激活，
        # 避免「Nav2 早于 spawn 启动 → 激活失败 → 小车不动」（尤其后启动的 agv2/agv3）。
        # on_exit 内按车号大幅错峰(2/17/32s)：3 套 Nav2 容器若同时加载组合节点，
        # load_node 服务会超时(实测 agv3 的 bt_navigator 因此加载失败→导航未就绪)，
        # 故每套 Nav2 间隔 ~15s 顺序加载。
        nodes.append(RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_node,
                on_exit=[TimerAction(period=2.0 + 15.0 * i, actions=nav2_actions)],
            )
        ))

    # 调度中心
    nodes.append(Node(
        package='amr_description', executable='fleet_manager', name='fleet_manager',
        parameters=[{
            'use_sim_time': True,
            'robot_namespaces': ns_list,
            'tasks_file': tasks_file,
            'battery_topic_type': 'battery_state',
            'charger_zone_names': ['charger_1'],
            'battery_low_threshold': 0.20,
            'battery_resume_threshold': 0.60,
            'goal_reach_dist': 0.30,
            'require_nav_ready': True,
            'world_file': world,
        }],
        output='screen', emulate_tty=True,
    ))

    # Web 面板层
    if use_web:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_web, 'launch', 'web_panel.launch.py'))))

    return nodes


def generate_launch_description():
    pkg_amr = get_package_share_directory('amr_description')
    default_world = os.path.join(pkg_amr, 'worlds', 'warehouse.world')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world 绝对路径。'),
        DeclareLaunchArgument('use_web', default_value='true',
                              description='是否启动 Web 面板层 (rosbridge + 网页)。'),
        DeclareLaunchArgument('num_agvs', default_value='2',
                              description='Nav2 车数 1-3。默认 2：本机 WSL2(8G/16核) 同时拉 3 套完整 Nav2 '
                                          '会出现第 3 套组合节点(load_node)加载超时、导航栈起不全，故默认 2 稳定可靠。'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 客户端 GUI (gzclient)。headless 压测设 false。'),
        OpaqueFunction(function=launch_setup),
    ])
