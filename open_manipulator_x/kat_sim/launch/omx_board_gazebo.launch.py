import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    open_manipulator_description_path = get_package_share_directory('kat_description')
    kat_sim_path = get_package_share_directory('kat_sim')

    # worlds
    worlds_path = os.path.join(kat_sim_path, 'gazebo', 'worlds')

    # model/mesh root
    models_root = str(Path(open_manipulator_description_path).parent.resolve())

    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[worlds_path, ':' + models_root],
    )
    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[worlds_path, ':' + models_root],
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Gazebo Sim world name (파일명, 확장자 제외)'
    )

    gz_args_str = [
        LaunchConfiguration('world'),
        TextSubstitution(text='.sdf -v 1 -r')
    ]

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
        ]),
        launch_arguments={'gz_args': gz_args_str}.items(),
    )

    # =========================
    # Robot description (xacro)
    # =========================
    robot_xacro_file = os.path.join(
        open_manipulator_description_path,
        'urdf',
        'open_manipulator_x',
        'open_manipulator_x.urdf.xacro'
    )
    robot_doc = xacro.process_file(robot_xacro_file, mappings={'use_sim': 'true'})
    robot_desc = robot_doc.toprettyxml(indent='  ')

    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', 'om',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            '-allow_renaming', 'true',
        ],
    )

    # =========================
    # Board (plain URDF)
    # =========================
    board_urdf_file = os.path.join(
        open_manipulator_description_path,
        'urdf',
        'board',
        'board_assembled.urdf'
    )
    with open(board_urdf_file, 'r') as f:
        board_desc = f.read()

    spawn_board = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', board_desc,
            '-name', 'board',
            '-x', '0.5', '-y', '0.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            '-allow_renaming', 'true',
        ],
    )

    # =========================
    # Controllers (spawner)
    # =========================
    # Gazebo 플러그인이 띄운 controller_manager를 쓰는 경우가 많아서
    # 기본은 /controller_manager로 두되, 필요하면 여기만 바꾸면 됨.
    controller_manager_name = '/controller_manager'

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', controller_manager_name],
        output='screen',
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', controller_manager_name],
        output='screen',
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', controller_manager_name],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen',
    )

    on_spawn_exit = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])
    )
    on_jsb_exit = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner, gripper_spawner])
    )

    return LaunchDescription([
        set_gz_resource,
        set_ign_resource,
        world_arg,
        gz_launch,
        state_pub,
        spawn_robot,
        spawn_board,
        on_spawn_exit,
        on_jsb_exit,
        clock_bridge,
    ])
