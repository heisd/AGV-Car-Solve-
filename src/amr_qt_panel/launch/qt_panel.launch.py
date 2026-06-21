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
