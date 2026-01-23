import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 시뮬레이션 시간 설정
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    # 2. MoveIt 설정 로드
    # .robot_description()에 인자를 넣는 대신 기본 로드 후 파라미터를 추가합니다.
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="kat_moveit_config")
        .to_moveit_configs()
    )
    
    # [핵심 수정] 딕셔너리의 update 메서드를 사용하여 안전하게 값을 주입합니다.
    # trajectory_execution 관련 파라미터는 보통 moveit_config.trajectory_execution 딕셔너리에 들어갑니다.
    if "trajectory_execution" in moveit_config.to_dict():
        moveit_config.trajectory_execution.update({"allowed_start_tolerance": 0.5})
    else:
        # 만약 해당 키가 없다면 직접 추가합니다.
        moveit_config.trajectory_execution = {"allowed_start_tolerance": 0.5}
        
    # moveit_config의 각 요소들에 use_sim_time 파라미터를 강제로 주입
    moveit_config.robot_description.update({"use_sim_time": True})
    moveit_config.robot_description_semantic.update({"use_sim_time": True})
    moveit_config.robot_description_kinematics.update({"use_sim_time": True})

    # 3. Gazebo + MoveGroup 런치 파일 실행
    bringup_dir = get_package_share_directory('kat_bringup')
    included_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'move_group_controller_gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 4. RViz 실행
    rviz_config_file = os.path.join(
        get_package_share_directory("kat_moveit_config"), "config", "moveit.rviz"
    )

    # 모든 MoveIt 관련 파라미터를 하나의 리스트로 통합
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {"use_sim_time": use_sim_time}, # RViz 시계를 흐르게 하는 핵심
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=rviz_parameters,
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        declare_use_sim_time,
        included_gazebo_launch,
        rviz_node
    ])