# warehouse_bringup.launch.py
#
# Top-level bring-up for the warehouse simulation:
#   - Gazebo Classic with our warehouse world (20 shelves + pickup + charging station)
#   - One AGV (diff-drive + 2D LiDAR) spawned in the aisle
#   - Per-robot sim helper nodes: odom_sim_filter, battery_sim, charger_dock_monitor
#   - Optional RViz2
#
# Usage:
#   ros2 launch amr_description warehouse_bringup.launch.py
#   ros2 launch amr_description warehouse_bringup.launch.py use_rviz:=true x:=-5.5 y:=-5.5
#   ros2 launch amr_description warehouse_bringup.launch.py world:=/abs/path/to/other.world
#
# Arguments:
#   namespace  (str,  default agv1)   robot namespace (frames/topics are prefixed with it)
#   world      (str,  default <pkg>/worlds/warehouse.world)  absolute path to the Gazebo world
#   x, y, z    (float, default 0.0/0.0/0.1)  spawn pose (default: central aisle)
#   use_rviz   (bool, default false)  launch RViz2
#
# Notes:
#   * Multi-AGV + Nav2 + fleet management lives in amr_fleet_management.launch.py.
#     That path needs a map of THIS world first (run mapping.launch.py / SLAM), so this
#     bring-up intentionally stops at "robot driveable in the warehouse".
#   * charger_centers_xy is set to this world's charging station pad (-5.5, -5.5).

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Ensure ~/.gazebo/models is in GAZEBO_MODEL_PATH so Gazebo can find the models
    gazebo_model_path = os.path.expanduser('~/.gazebo/models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        if gazebo_model_path not in os.environ['GAZEBO_MODEL_PATH'].split(os.pathsep):
            os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    pkg_amr = get_package_share_directory('amr_description')
    default_world = os.path.join(pkg_amr, 'worlds', 'warehouse.world')

    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_args = [
        DeclareLaunchArgument('namespace', default_value='agv1',
                              description='Robot namespace.'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Absolute path to the Gazebo world file.'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X (m).'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y (m).'),
        DeclareLaunchArgument('z', default_value='0.1', description='Spawn Z (m).'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz2.'),
    ]

    xacro_file = PathJoinSubstitution([FindPackageShare('amr_description'), 'urdf', 'amr.urdf.xacro'])
    rviz_config = PathJoinSubstitution([FindPackageShare('amr_description'), 'rviz', 'amr_config.rviz'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' namespace:=', namespace]), value_type=str
    )

    # Gazebo Classic + our warehouse world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_amr',
        output='screen',
        arguments=['-entity', namespace, '-topic', 'robot_description', '-x', x, '-y', y, '-z', z],
    )

    odom_sim_filter = Node(
        package='amr_description',
        executable='odom_sim_filter',
        name='odom_sim_filter',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_namespace': namespace}],
    )

    battery_sim = Node(
        package='amr_description',
        executable='battery_sim',
        name='battery_sim',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_namespace': namespace}],
    )

    # Charger proximity monitor — pointed at this world's charging station pad (-5.5, -5.5)
    charger_dock_monitor = Node(
        package='amr_description',
        executable='charger_dock_monitor',
        name='charger_dock_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'robot_namespace': namespace,
            'charger_centers_xy': [-5.5, -5.5],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declare_args + [
        gazebo,
        robot_state_publisher,
        spawn_entity,
        odom_sim_filter,
        battery_sim,
        charger_dock_monitor,
        rviz,
    ])
