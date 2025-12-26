import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from pymoveit2 import MoveIt2

class ComputerMoveListener(Node):
    def __init__(self):
        super().__init__('computer_move_listener')

        # 1. 파라미터 선언 (기본값을 부여하여 선언만 해둡니다)
        # YAML에서 값을 읽어오지 못할 경우를 대비해 빈 리스트를 기본값으로 설정합니다.
        for i in range(9):
            self.declare_parameter(f"cell_{i}", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home", [0.0, 0.0, 0.0, 0.0])

        # 2. MoveIt2 세팅 (로봇 설정에 맞게 수정 필요)
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

        # 3. 구독 설정
        self.create_subscription(
            Int8,
            '/kat/computer_move',
            self.computer_move_callback,
            10
        )

        self.get_logger().info("=== Computer Move Listener 가동 준비 완료 ===")

    def computer_move_callback(self, msg):
        move = int(msg.data)
        if move < 0 or move > 8:
            self.get_logger().error(f"잘못된 위치 번호: {move} (0~8만 가능)")
            return

        # 해당 칸의 파라미터 키 이름 생성
        key = f"cell_{move}"
        
        # 파라미터 서버에서 실시간으로 값 가져오기
        # .value를 통해 직접 리스트(float) 형태를 얻습니다.
        angles = self.get_parameter(key).value

        # 데이터 유효성 검사
        if not angles or len(angles) == 0:
            self.get_logger().error(f"파라미터 '{key}'를 읽었으나 비어있습니다. YAML 로드를 확인하세요.")
            return

        if len(angles) != len(self.joint_names):
            self.get_logger().error(
                f"'{key}' 설정값 개수({len(angles)})가 joint_names 개수({len(self.joint_names)})와 다릅니다."
            )
            return

        self.get_logger().info(f"이동 명령 수신: 칸 번호 {move} -> 각도 {angles}")

        # 4. MoveIt2 동작 실행
        try:
            # 로봇 팔 이동 명령
            self.moveit2.move_to_configuration(angles)
            
            # 실행이 완료될 때까지 대기 (Blocking)
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
