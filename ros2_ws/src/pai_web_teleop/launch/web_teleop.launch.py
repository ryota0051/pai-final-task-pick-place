"""rosbridge + 手先テレオペ + 録画ノード + HTML UI 配信をまとめて起動する."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pai_web_teleop')
    rosbridge_share = get_package_share_directory('rosbridge_server')

    args = [
        DeclareLaunchArgument('teleop_config', default_value=os.path.join(share, 'config', 'teleop.yaml')),
        DeclareLaunchArgument('recorder_config', default_value=os.path.join(share, 'config', 'recorder.yaml')),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('web_dir', default_value=os.path.join(share, 'web')),
    ]

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')),
        launch_arguments={'port': LaunchConfiguration('rosbridge_port')}.items(),
    )
    teleop = Node(
        package='pai_web_teleop', executable='ee_teleop_node', output='screen',
        parameters=[LaunchConfiguration('teleop_config')],
    )
    recorder = Node(
        package='pai_web_teleop', executable='recorder_node', output='screen',
        parameters=[LaunchConfiguration('recorder_config')],
    )
    http = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', LaunchConfiguration('http_port'),
             '--directory', LaunchConfiguration('web_dir')],
        output='screen',
    )
    return LaunchDescription(args + [rosbridge, teleop, recorder, http])
