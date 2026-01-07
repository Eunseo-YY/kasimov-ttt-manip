import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2
import threading
import time
from shape_msgs.msg import Mesh
from geometry_msgs.msg import Pose

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
        # 3) MoveIt2 초기화(팔)
        # ----------------------------
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4"],
            base_link_name="joint0",   
            end_effector_name="hand", 
            group_name="arm",
        )
        self.get_logger().info("MoveIt2 초기화 성공")
        
        # ----------------------------
        # 3-1) STL 장애물 추가 (MoveIt2 초기화 직후 수행)
        # ----------------------------
        self.add_stl_mesh()

        # ----------------------------
        # 3-2) MoveIt2 초기화(그리퍼)
        # ----------------------------
        self.gripper = MoveIt2(
            node=self,
            joint_names=["gripper_left_joint"],
            base_link_name="joint0",
            end_effector_name="hand",
            group_name="hand",
        )
        
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
    # 신규: STL 장애물 추가 함수
    # ----------------------------
    def add_stl_mesh(self):
        """
        Planning Scene에 STL 파일을 장애물로 등록합니다.
        """
        # 1. 경로 설정 (확장자 .stl 확인 필수!)
        raw_path = "~/ws_kat/src/kasimov-ttt-manip/open_manipulator_x/kat_description/meshes/board/board_assembled.stl"
        stl_path = os.path.expanduser(raw_path)
        
        # 2. 위치 및 자세 설정
        obj_pose = Pose()
        # 로봇 기준 좌표(joint0)에서의 위치 (필요에 따라 x, y, z 조정)
        obj_pose.position.x = 0.0  # 로봇 앞으로 20cm 이동 예시
        obj_pose.position.y = 0.0
        obj_pose.position.z = 0.0
        
        # 3. 중요: 쿼터니언 값의 기본값(회전 없음)은 w=1.0 입니다.
        obj_pose.orientation.x = 0.0
        obj_pose.orientation.y = 0.0
        obj_pose.orientation.z = 0.0
        obj_pose.orientation.w = 1.0

        try:
            # pymoveit2의 add_mesh를 사용하여 장애물 등록
            self.moveit2.add_mesh(
                name="board",
                pose=obj_pose,
                file_path=stl_path
            )
            self.get_logger().info(f"✅ STL 장애물 추가 완료: {stl_path}")
        except Exception as e:
            self.get_logger().error(f"❌ STL 장애물 추가 중 에러 발생: {e}")

    # ----------------------------
    # 유틸: 하트비트(선택)
    # ----------------------------
    def heartbeat(self):
        while rclpy.ok():
            # print 대신 get_logger를 사용하는 것이 좋습니다.
            # self.get_logger().debug("[Heartbeat] Node is spinning...")
            time.sleep(2.0)

    # ----------------------------
    # 핵심: /joint_states가 들어오면 home 1회 실행
    # ----------------------------
    def joint_states_cb(self, msg: JointState):
        if not self.joint_states_ready:
            self.joint_states_ready = True
            self.get_logger().info("✅ joint_states 첫 수신 완료")

        if self.joint_states_ready and (not self.home_done):
            self.home_done = True 
            threading.Thread(target=self.move_to_home_once, daemon=True).start()

    def wait_executed_with_timeout(self, timeout_s=10.0, period_s=0.1) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.moveit2.wait_until_executed():
                return True
            time.sleep(period_s)
        return False

    def move_to_home_once(self):
        if self.is_moving:
            return
        home = self.get_parameter("home").value
        self.get_logger().info(f"▶ 초기 HOME 이동 시작: {home}")
        self.is_moving = True
        try:
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

    def control_gripper(self, open_mode=True):
        target_val = [0.019] if open_mode else [0.0]
        try:
            self.gripper.move_to_configuration(target_val)
            time.sleep(1.0) 
        except Exception as e:
            self.get_logger().error(f"그리퍼 조작 에러: {e}")

    def computer_move_callback(self, msg: Int8):
        move_id = int(msg.data)
        self.get_logger().info(f"● 토픽 수신 (ID: {move_id})")
        if not self.home_done or self.is_moving:
            self.get_logger().warn("준비되지 않음 또는 이동 중 → 명령 무시")
            return
        if move_id < 0 or move_id > 8:
            return
        threading.Thread(target=self.execute_move_task, args=(move_id,), daemon=True).start()

    def execute_move_task(self, move_id: int):
        self.is_moving = True
        key = f"cell_{move_id}"
        angles = self.get_parameter(key).value
        self.get_logger().info(f"▶ {key} 이동 시도: {angles}")
        try:
            self.control_gripper(open_mode=True)
            self.moveit2.move_to_configuration(angles)
            ok = self.wait_executed_with_timeout(timeout_s=10.0)
            if ok:
                self.control_gripper(open_mode=False)
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