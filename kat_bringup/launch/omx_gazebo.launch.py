#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Wonho Yun, Sungho Woo

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def _default_world_path():
    # 표준 경로 후보들
    candidates = [
        "/usr/share/ignition/gazebo6/worlds/empty.sdf",  # Fortress 계열
        "/usr/share/gz/gazebo/worlds/empty.sdf",         # 신규 네이밍
        "/usr/share/ignition/gazebo/worlds/empty.sdf",   # 기타
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    # 마지막 수단: 패키지 안 worlds/empty_world.sdf 가 있으면 사용
    try:
        bringup_share = get_package_share_directory('kat_bringup')
        local_world = os.path.join(kat_bringup_share, 'worlds', 'empty_world.sdf')
        if os.path.exists(local_world):
            return local_world
    except Exception:
        pass
    # 못 찾으면 그냥 첫 후보 반환(에러 메시지라도 명확)
    return candidates[0]

def generate_launch_description():
    # Launch Arguments
    open_manipulator_description_path = os.path.join(
        get_package_share_directory('kat_description')
    )

    open_manipulator_bringup_path = os.path.join(
        get_package_share_directory('kat_bringup')
    )

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(open_manipulator_bringup_path, 'worlds'),
            ':' + str(Path(open_manipulator_description_path).parent.resolve()),
        ],
    )
    set_ign_res = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f"{os.path.join(open_manipulator_bringup_path, 'worlds')}:{Path(open_manipulator_description_path).parent.resolve()}"
    )

    default_world = _default_world_path()
    arguments = LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=default_world, description='Absolute path to world SDF'
        ),
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), ' -v 1', ' -r'])
        ],
    )

    xacro_file = os.path.join(
        open_manipulator_description_path,
        'urdf',
        'open_manipulator_x',
        'open_manipulator_x.urdf.xacro',
    )

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            robot_desc,
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0',
            '-R',
            '0.0',
            '-P',
            '0.0',
            '-Y',
            '0.0',
            '-name',
            'om',
            '-allow_renaming',
            'true',
        ],
    )

    # Controller spawner nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen',
    )

    # rviz_config_file = os.path.join(
    #     open_manipulator_description_path, 'rviz', 'open_manipulator.rviz'
    # )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='log',
    #     arguments=['-d', rviz_config_file],
    # )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        bridge,
        gazebo_resource_path,
        set_ign_res, 
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        # rviz,
    ])