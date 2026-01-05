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
    # Package paths
    open_manipulator_description_path = get_package_share_directory('kat_description')
    open_manipulator_bringup_path = get_package_share_directory('kat_bringup')
    kat_sim_path = get_package_share_directory('kat_sim')

    # ✅ worlds 경로: kat_sim에 있는 empty_world.sdf를 쓰게 만들기
    # 네 구조: kat_sim/gazebo/worlds/empty_world.sdf
    worlds_path = os.path.join(kat_sim_path, 'gazebo', 'worlds')

    # 모델 리소스 루트(메시/모델 탐색용)
    models_root = str(Path(open_manipulator_description_path).parent.resolve())

    # ✅ gz/ign 둘 다 worlds_path 포함
    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[worlds_path, ':' + models_root],
    )
    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[worlds_path, ':' + models_root],
    )

    # Launch args (파일명만 받음)
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Gazebo Sim world name (파일명, 확장자 제외)'
    )

    # ✅ gz_args를 "단일 문자열"로 넘기기 (진짜 안정적)
    gz_args_str = [
        LaunchConfiguration('world'),
        TextSubstitution(text='.sdf -v 1 -r')
    ]

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
        ]),
        launch_arguments={
            'gz_args': gz_args_str
        }.items(),
    )

    # Robot description (use_sim=true)
    xacro_file = os.path.join(
        open_manipulator_description_path, 'urdf', 'open_manipulator_x', 'open_manipulator_x.urdf.xacro'
    )
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    spawn = Node(
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

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen',
    )

    on_spawn_exit = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
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
        spawn,
        on_spawn_exit,
        on_jsb_exit,
        clock_bridge,
    ])
