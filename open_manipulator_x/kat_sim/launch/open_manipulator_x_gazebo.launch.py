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
    kat_sim_path = get_package_share_directory('kat_sim')

    worlds_path = os.path.join(kat_sim_path, 'gazebo', 'worlds')
    models_root = str(Path(open_manipulator_description_path).parent.resolve())

    set_gz_resource = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[worlds_path, ':' + models_root])
    set_ign_resource = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[worlds_path, ':' + models_root])

    world_arg = DeclareLaunchArgument('world', default_value='empty_world')

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
        ]),
        launch_arguments={'gz_args': [LaunchConfiguration('world'), '.sdf -v 1 -r']}.items(),
    )

    # Robot description
    xacro_file = os.path.join(open_manipulator_description_path, 'urdf', 'open_manipulator_x', 'open_manipulator_x.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')

    # ✅ 1. 이전에 잘 작동하던 방식으로 state_pub 파라미터 복구
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc, '-name', 'om', '-allow_renaming', 'true'],
    )

    # ✅ 2. Clock Bridge: Ignition 6(Fortress)에서 가장 표준적인 형식으로 수정
    # 대괄호 '[' 의 위치와 메시지 타입을 확실히 합니다.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    # Spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
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

    # ✅ 3. 이벤트 핸들러: Spawn이 완전히 끝난 후 컨트롤러 실행
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