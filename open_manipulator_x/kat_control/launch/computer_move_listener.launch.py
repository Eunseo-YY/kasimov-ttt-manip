#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 기본 params 파일 경로(패키지 안의 config로 잡아둠)
    default_params = PathJoinSubstitution([
        FindPackageShare("kat_control"),
        "config",
        "open_manipulator_x",
        "positions.yaml",
    ])

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML params file for computer_move_listener (cell_0~8, home).",
    )

    listener_node = Node(
        package="kat_control",
        executable="computer_move_listener",
        name="computer_move_listener",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        listener_node,
    ])
