import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int8
from pymoveit2 import MoveIt2
import sys

class ComputerMoveListener(Node):
    def __init__(self):
        super().__init__('computer_move_listener')

        # 1. 콜백 그룹 설정 (멀티스레딩을 위해 필수)
        self.callback_group = ReentrantCallbackGroup()

        # 2. 파라미터 선언
        for i in range(9):
            self.declare_parameter(f"cell_{i}", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home", [0.0, 0.0, 0.0, 0.0])

        # 3. MoveIt2 세팅 (로봇 설정에 맞게 수정됨)
        self.group_name = "arm" 
        self.joint_names = ["joint1", "joint2", "joint3", "joint4"] 
        self.base_link_name = "joint0" 
        self.end_effector_name = "hand"

        try:
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=self.joint_names,
                base_link_name=self.base_link_name,
                end_effector_name=self.end_effector_name,
                group_name=self.group_name,
            )
        except Exception as e:
            self.get_logger().error(f"MoveIt2 초기화 실패: {e}")

        # 4. 구독 설정 (callback_group을 지정하여 멀티스레드 대응)
        self.create_subscription(
            Int8,
            '/kat/computer_move',
            self.computer_move_callback,
            10,
            callback_group=self.callback_group
        )

        self.get_logger().info("=== Computer Move Listener 가동 준비 완료 ===")

    def computer_move_callback(self, msg):
        move = int(msg.data)
        if move < 0 or move > 8:
            self.get_logger().error(f"잘못된 위치 번호: {move}")
            return

        key = f"cell_{move}"
        angles = self.get_parameter(key).value

        if not angles or len(angles) != len(self.joint_names):
            self.get_logger().error(f"'{key}' 파라미터 설정이 잘못되었습니다.")
            return

        self.get_logger().info(f"이동 명령 수신: 칸 번호 {move} -> 각도 {angles}")

        try:
            # MoveIt2 동작 실행
            self.moveit2.move_to_configuration(angles)
            success = self.moveit2.wait_until_executed()

            if success:
                self.get_logger().info(f"성공적으로 {key} 지점으로 이동했습니다.")
            else:
                self.get_logger().error(f"{key} 이동 실행에 실패했습니다.")
        except Exception as e:
            self.get_logger().error(f"동작 실행 중 예외 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ComputerMoveListener()
    
    # 멀티스레드 실행기 사용 (콜백과 MoveIt 동작이 동시에 돌아가게 함)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
