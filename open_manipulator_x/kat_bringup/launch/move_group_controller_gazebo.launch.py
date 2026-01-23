import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. MoveIt 설정 빌드 (패키지 이름: kat_moveit_config)
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="kat_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 2. move_group 노드 직접 정의
    # moveit_config.to_dict()를 통해 모든 설정을 가져오고, use_sim_time을 추가합니다.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
            "use_sim_time": True,  # Gazebo 시뮬레이션 시간 동기화 필수 설정
            "default_robot_padding": 0.0,    # 로봇 팔 주변의 가상 보호막 제거
            "default_object_padding": 0.0,   # 장애물(보드) 주변의 가상 보호막 제거
            "trajectory_execution.allowed_start_tolerance": 0.05, # 오차 허용 범위를 0.05로 확대
            "start_state_max_bounds_error": 0.1,
            }
        ],
    )

    return LaunchDescription([
        move_group_node
    ])