# web_panel.launch.py
#
# Brings up the web operator panel layer:
#   - rosbridge_server (WebSocket)  : browser <-> ROS 2 bridge on :<rosbridge_port>
#   - a static HTTP server          : serves the panel (index.html) on :<web_port>
#
# Open the panel at:  http://<host>:<web_port>/   (defaults to http://localhost:8080/)
# The panel talks ONLY to the dispatch center (fleet_manager) via /fleet/state and /fleet/add_task.
#
# Arguments:
#   rosbridge_port (int, default 9090)
#   web_port       (int, default 8080)
#   address        (str, default 0.0.0.0)  bind address for the HTTP server (0.0.0.0 = reachable from LAN)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory('amr_web'), 'web')
    rosbridge_launch = os.path.join(
        get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'
    )

    rosbridge_port = LaunchConfiguration('rosbridge_port')
    web_port = LaunchConfiguration('web_port')
    address = LaunchConfiguration('address')

    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='rosbridge WebSocket port.'),
        DeclareLaunchArgument('web_port', default_value='8080',
                              description='Static HTTP server port for the panel.'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='HTTP server bind address (0.0.0.0 = LAN reachable).'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
            launch_arguments={'port': rosbridge_port}.items(),
        ),

        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', web_port, '--bind', address, '--directory', web_dir],
            output='screen',
        ),

        LogInfo(msg=['[amr_web] Web panel: http://localhost:', web_port,
                     '/   (rosbridge ws://localhost:', rosbridge_port, ')']),
    ])
