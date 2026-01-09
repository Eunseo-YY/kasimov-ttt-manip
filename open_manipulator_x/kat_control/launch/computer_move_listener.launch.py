#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 기본 params 파일 경로
    default_params = PathJoinSubstitution([
        FindPackageShare("kat_control"),
        "config",
        "open_manipulator_x",
        "positions.yaml",
    ])
    
    # 1. 인자 선언 (터미널에서 use_sim_time:=true를 받기 위함)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML params file for computer_move_listener (cell_0~8, home).",
    )

    # 2. 노드 설정
    listener_node = Node(
        package="kat_control",
        executable="computer_move_listener",
        name="computer_move_listener",
        output="screen",
        # 핵심: 리스트 안에 YAML 파일과 use_sim_time을 함께 넣습니다.
        parameters=[
            LaunchConfiguration("params_file"),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,  # 인자 추가
        params_file_arg,
        listener_node,
    ])