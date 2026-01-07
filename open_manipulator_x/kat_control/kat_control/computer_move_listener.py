import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2
import threading
import time


class ComputerMoveListener(Node):
    def __init__(self):
        super().__init__("computer_move_listener")

        self.get_logger().info("=== 노드 시작 ===")

        # ----------------------------
        # 0) 내부 상태 플래그
        # ----------------------------
        self.is_moving = False
        self.joint_states_ready = False
        self.home_done = False

        # ----------------------------
        # 1) 파라미터 선언
        # ----------------------------
        for i in range(9):
            self.declare_parameter(f"cell_{i}", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home", [0.0, 0.0, 0.0, 0.0])

        # ----------------------------
        # 2) /joint_states 구독 (home 실행 트리거용)
        # ----------------------------
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 10)

        # ----------------------------
        # 3) MoveIt2 초기화
        # ----------------------------
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4"],
            base_link_name="joint0",   # 은서가 맞다고 했으니 유지 🙂
            end_effector_name="hand",  # 은서가 문제 아니라고 했으니 유지 🙂
            group_name="arm",
        )
        self.get_logger().info("MoveIt2 초기화 성공")

        # ----------------------------
        # 4) 명령 토픽 구독
        # ----------------------------
        self.create_subscription(
            Int8,
            "/kat/computer_move",
            self.computer_move_callback,
            10,
        )

        # ----------------------------
        # 5) 노드 생존 확인(선택)
        # ----------------------------
        threading.Thread(target=self.heartbeat, daemon=True).start()

        self.get_logger().info("=== 초기화 완료, joint_states 기다리는 중 ===")

    # ----------------------------
    # 유틸: 하트비트(선택)
    # ----------------------------
    def heartbeat(self):
        while rclpy.ok():
            print("[Heartbeat] Node is spinning...")
            time.sleep(2.0)

    # ----------------------------
    # 핵심: /joint_states가 들어오면 home 1회 실행
    # ----------------------------
    def joint_states_cb(self, msg: JointState):
        if not self.joint_states_ready:
            self.joint_states_ready = True
            self.get_logger().info("✅ joint_states 첫 수신 완료")

        # home은 딱 1번만
        if self.joint_states_ready and (not self.home_done):
            self.home_done = True  # 중복 실행 방지(먼저 올려두는 게 안전)
            threading.Thread(target=self.move_to_home_once, daemon=True).start()

    # ----------------------------
    # wait_until_executed() 타임아웃 래퍼
    # (너 환경에서는 timeout 인자 지원 안 하니까 직접 구현)
    # ----------------------------
    def wait_executed_with_timeout(self, timeout_s=10.0, period_s=0.1) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.moveit2.wait_until_executed():
                return True
            time.sleep(period_s)
        return False

    # ----------------------------
    # 시작 시 home 1회 이동
    # ----------------------------
    def move_to_home_once(self):
        if self.is_moving:
            return

        home = self.get_parameter("home").value
        self.get_logger().info(f"▶ 초기 HOME 이동 시작: {home}")

        self.is_moving = True
        try:
            # joint_states 들어온 직후에도 약간 안정화 시간 주는 게 실로봇에서 유리함
            time.sleep(0.5)

            self.moveit2.move_to_configuration(home)

            ok = self.wait_executed_with_timeout(timeout_s=10.0)
            if ok:
                self.get_logger().info("✅ HOME 이동 완료")
            else:
                self.get_logger().error("❌ HOME 이동 실패 (timeout)")

        except Exception as e:
            self.get_logger().error(f"⚠️ HOME 이동 중 예외: {e}")
        finally:
            self.is_moving = False
            self.get_logger().info("=== HOME 처리 종료, 명령 대기 ===")

    # ----------------------------
    # cell 명령 수신 콜백
    # ----------------------------
    def computer_move_callback(self, msg: Int8):
        move_id = int(msg.data)
        self.get_logger().info(f"● 토픽 수신 (ID: {move_id})")

        # home 끝나기 전이면 명령 무시(원하면 큐잉도 가능하지만 일단 단순하게)
        if not self.home_done:
            self.get_logger().warn("HOME 아직 실행 전/중 → 명령 무시")
            return

        if self.is_moving:
            self.get_logger().warn("이동 중 → 명령 무시")
            return

        # 범위 체크(방어)
        if move_id < 0 or move_id > 8:
            self.get_logger().error(f"ID 범위 오류: {move_id} (0~8만 허용)")
            return

        threading.Thread(target=self.execute_move_task, args=(move_id,), daemon=True).start()

    # ----------------------------
    # 실제 이동 수행
    # ----------------------------
    def execute_move_task(self, move_id: int):
        self.is_moving = True
        key = f"cell_{move_id}"
        angles = self.get_parameter(key).value

        self.get_logger().info(f"▶ {key} 이동 시도: {angles}")

        try:
            self.moveit2.move_to_configuration(angles)

            ok = self.wait_executed_with_timeout(timeout_s=10.0)
            if ok:
                self.get_logger().info(f"✅ {key} 이동 성공")
            else:
                self.get_logger().error(f"❌ {key} 이동 실패 (timeout)")

        except Exception as e:
            self.get_logger().error(f"⚠️ 이동 중 예외: {e}")
        finally:
            self.is_moving = False


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


if __name__ == "__main__":
    main()
