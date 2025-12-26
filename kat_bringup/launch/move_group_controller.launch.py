from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("open_manipulator_x", package_name="kat_moveit_config")
        # ↓↓↓ 이 한 줄이 핵심이야 (MoveIt 컨트롤러 매핑을 move_group에 넣음) 🙂
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    return generate_move_group_launch(moveit_config)
