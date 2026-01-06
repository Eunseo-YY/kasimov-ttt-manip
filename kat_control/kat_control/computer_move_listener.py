import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int8
from pymoveit2 import MoveIt2
import threading
import time

class ComputerMoveListener(Node):
    def __init__(self):
        super().__init__('computer_move_listener')
        
        self.get_logger().info("=== [비동기 강화 버전] 노드 시작 ===")

        # 1. 생존 확인용 Heartbeat (별도 스레드)
        threading.Thread(target=self.raw_python_heartbeat, daemon=True).start()

        # 2. 파라미터 선언
        for i in range(9):
            self.declare_parameter(f"cell_{i}", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home", [0.0, 0.0, 0.0, 0.0])

        # 3. MoveIt2 초기화
        try:
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=["joint1", "joint2", "joint3", "joint4"],
                base_link_name="joint0",
                end_effector_name="hand",
                group_name="arm",
            )
            self.get_logger().info("MoveIt2 초기화 성공")
        except Exception as e:
            self.get_logger().error(f"초기화 에러: {e}")

        # 4. 구독 설정 (가장 단순하게 유지)
        self.create_subscription(
            Int8, 
            '/kat/computer_move', 
            self.computer_move_callback, 
            10
        )
        
        self.is_moving = False # 현재 이동 중인지 확인하는 플래그
        self.get_logger().info("=== 모든 준비 완료: 명령 대기 중 ===")

    def raw_python_heartbeat(self):
        while rclpy.ok():
            print("[Heartbeat] Node is spinning...")
            time.sleep(2.0)

    def computer_move_callback(self, msg):
        """토픽을 받으면 실행되는 함수"""
        move_id = int(msg.data)
        self.get_logger().info(f"● 토픽 수신 확인 (ID: {move_id})")

        if self.is_moving:
            self.get_logger().warn("현재 로봇이 이동 중입니다. 이번 명령은 무시됩니다.")
            return

        # [핵심] 실제 이동 처리는 별도의 스레드에 맡기고 콜백은 즉시 종료합니다.
        # 이렇게 해야 ROS 2 실행기가 다음 토픽을 받을 수 있습니다.
        task_thread = threading.Thread(
            target=self.execute_move_task, 
            args=(move_id,), 
            daemon=True
        )
        task_thread.start()

    def execute_move_task(self, move_id):
        """실제 이동을 수행하는 별도 스레드 함수"""
        self.is_moving = True
        key = f"cell_{move_id}"
        angles = self.get_parameter(key).value
        
        self.get_logger().info(f"▶ [스레드 시작] {key} 이동 시도 (각도: {angles})")
        
        try:
            # 1. 이동 명령 전송
            self.moveit2.move_to_configuration(angles)
            
            # 2. 완료 대기 (최대 10초)
            # wait_until_executed가 여기서 스레드를 붙잡아도 ROS2 메인 실행기에는 영향을 주지 않습니다.
            success = False
            for _ in range(100):
                if self.moveit2.wait_until_executed():
                    success = True
                    break
                time.sleep(0.1)
            
            if success:
                self.get_logger().info(f"✅ [스레드 종료] {key} 이동 성공")
            else:
                self.get_logger().error(f"❌ [스레드 종료] {key} 이동 실패 (타임아웃)")
                
        except Exception as e:
            self.get_logger().error(f"⚠️ 이동 중 예외 발생: {e}")
        finally:
            self.is_moving = False # 이동 완료 후 플래그 해제

def main(args=None):
    rclpy.init(args=args)
    node = ComputerMoveListener()
    
    # 안정적인 싱글 스레드 실행기를 사용해도 비동기 스레드 덕분에 잘 돌아갑니다.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()