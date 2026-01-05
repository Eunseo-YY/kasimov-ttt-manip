#!/usr/bin/env python3
#
# Minimal real-hardware launch for OpenManipulator-X (ROS2)
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix of the joint and link names',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Port name for hardware connection (e.g., /dev/ttyUSB0)',
        ),
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='open_manipulator_x',
            description='Type of ros2_control',
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Whether to run robot_state_publisher (TF)',
        ),
    ]

    prefix = LaunchConfiguration('prefix')
    port_name = LaunchConfiguration('port_name')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    publish_tf = LaunchConfiguration('publish_tf')

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('kat_description'),
            'urdf',
            'open_manipulator_x',
            'open_manipulator_x.urdf.xacro',
        ]),
        ' ',
        'prefix:=', prefix,
        ' ',
        'use_sim:=false',
        ' ',
        'use_fake_hardware:=false',
        ' ',
        'fake_sensor_commands:=false',
        ' ',
        'port_name:=', port_name,
        ' ',
        'ros2_control_type:=', ros2_control_type,
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('kat_control'),
        'config',
        'open_manipulator_x',
        'hardware_controller_manager.yaml',
    ])

    # ✅ 컨트롤러 매니저(ros2_control_node) 노드 이름을 고정
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            {
                'robot_description': ParameterValue(
                    urdf_file,
                    value_type=str
                )
            },
            controller_manager_config
        ],
        output='both',
    )


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(
                    urdf_file,
                    value_type=str
                ),
                'use_sim_time': False
            }
        ],
        output='both',
        condition=IfCondition(publish_tf),
    )


    # ✅ spawner들이 정확히 이 컨트롤러 매니저를 바라보게 명시
 #   spawn_controllers = TimerAction(
#      period=2.0,
#        actions=[
 #           Node(
  #              package='controller_manager',
#             executable='spawner',
#                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
#                output='both',
#            ),
#            Node(
#                package='controller_manager',
#                executable='spawner',
#                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
#                output='both',
#            ),
#            Node(
#                package='controller_manager',
#                executable='spawner',
#                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
#                output='both',
#            ),
 #       ],
 #   )

    return LaunchDescription(
        declared_arguments + [control_node, robot_state_publisher_node]
    )
