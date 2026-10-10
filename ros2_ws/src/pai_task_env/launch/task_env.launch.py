"""把持・配置タスク用の Gazebo 環境（CRANE+ V2 + テーブル + 物体 + 置き場所）とタスク管理ノードを起動する.

演習イメージの crane_plus_gazebo/crane_plus_with_table.launch.py は world が table.sdf に固定されているため、
同等の構成（ign gazebo + spawn + ros2_control spawner + clock bridge）をここで組み直し、
任意の world を読み込めるようにしている。MoveIt / RViz は不要なので起動しない。
"""
import os

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    share = get_package_share_directory('pai_task_env')
    gazebo_share = get_package_share_directory('crane_plus_gazebo')

    args = [
        DeclareLaunchArgument('world_name', default_value=os.path.join(share, 'worlds', 'pick_place.sdf')),
        DeclareLaunchArgument('task_config', default_value=os.path.join(share, 'config', 'task.yaml')),
        DeclareLaunchArgument('robot_z', default_value='1.015', description='ロボットのスポーン高さ（task.yaml の base_origin と一致させる）'),
        DeclareLaunchArgument('object_odom_gz_topic', default_value='/model/target_cube/odometry'),
    ]

    # PATH を追加で通さないと STL が読み込まれない（元 launch と同じ）
    env = {
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ.get('LD_LIBRARY_PATH', ''),
        'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(get_package_share_directory('crane_plus_description')),
    }
    gui_config = os.path.join(gazebo_share, 'gui', 'gui.config')
    # -r で起動時にシミュレーションを開始しないとコントローラが起動しない
    ign_gazebo = ExecuteProcess(
        cmd=['ign gazebo -r', LaunchConfiguration('world_name'), '--gui-config', gui_config],
        output='screen', additional_env=env, shell=True,
    )

    description_loader = RobotDescriptionLoader()
    # 把持アシスト用 DetachableJoint を追加したラッパー xacro を使う
    description_loader.robot_description_path = os.path.join(share, 'urdf', 'crane_plus_task.urdf.xacro')
    description_loader.use_gazebo = 'true'
    description_loader.use_ignition = 'true'
    description_loader.gz_control_config_package = 'crane_plus_control'
    description_loader.gz_control_config_file_path = 'config/crane_plus_controllers.yaml'
    description = description_loader.load()

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': description}],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/robot_description', '-name', 'crane_plus',
                   '-z', LaunchConfiguration('robot_z'), '-allow_renaming', 'true'],
    )
    spawners = [
        ExecuteProcess(cmd=[f'ros2 run controller_manager spawner {c}'], shell=True, output='screen')
        for c in ('joint_state_broadcaster', 'crane_plus_arm_controller', 'crane_plus_gripper_controller')
    ]

    odom_topic = LaunchConfiguration('object_odom_gz_topic')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='task_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   [odom_topic, '@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
                   '/task/gz/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                   '/task/gz/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                   '/task/gz/attach_state@std_msgs/msg/String[ignition.msgs.StringMsg'],
        remappings=[(odom_topic, '/task/object_odom')],
    )

    task_manager = Node(
        package='pai_task_env', executable='task_manager_node', output='screen',
        parameters=[LaunchConfiguration('task_config')],
    )
    return LaunchDescription(args + [
        SetParameter(name='use_sim_time', value=True),
        ign_gazebo, robot_state_publisher, spawn, *spawners, bridge, task_manager,
    ])
