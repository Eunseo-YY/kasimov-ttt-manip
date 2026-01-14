import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2
from geometry_msgs.msg import PoseStamped, Point, Pose
import threading
import time
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
import trimesh
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject
from std_msgs.msg import Header


class ComputerMoveListener(Node):
    def __init__(self):
        super().__init__("computer_move_listener")

        self.get_logger().info(f"Sim Time 사용 여부: {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")
        # [추가] 시뮬레이션 오차 허용 범위 강제 설정 (Invalid Trajectory 에러 방지)
        os.system("ros2 param set /move_group trajectory_execution/allowed_start_tolerance 0.5")
        self.get_logger().info("=== 노드 시작 ===")
        
        # ----------------------------
        # 0) 내부 상태 플래그
        # ----------------------------
        self.is_moving = False
        self.joint_states_ready = False
        self.home_done = False
        self.obstacle_added = False

        # ----------------------------
        # 1) 파라미터 선언
        # ----------------------------
        for i in range(9):
            self.declare_parameter(f"cell_{i}", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("pick", [3.14159265, -0.78539816, 0.52359878, 1.57079633])
        self.declare_parameter("loading_zone", [0.0, 0.0, 0.0, 0.0])


        # ----------------------------
        # 2) /joint_states 구독
        # ----------------------------
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_cb,
            10
        )

        # ----------------------------
        # 3) MoveIt2 인터페이스 초기화
        # ----------------------------
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4"],
            base_link_name="joint0",
            end_effector_name="hand",
            group_name="arm",
        )

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

        threading.Thread(target=self.heartbeat, daemon=True).start()
        self.get_logger().info("=== 초기화 완료 ===")

    # ----------------------------
    # joint_states 콜백
    # ----------------------------
    def joint_states_cb(self, msg: JointState):
        if not self.joint_states_ready:
            self.joint_states_ready = True
            self.get_logger().info("joint_states 첫 수신")

            #if not self.obstacle_added:
                #self.setup_obstacles()

        if self.joint_states_ready and not self.home_done:
            self.home_done = True
            threading.Thread(
                target=self.move_to_home_once,
                daemon=True
            ).start()

    # ----------------------------
    # HOME 이동
    # ----------------------------
    def move_to_home_once(self):
        if self.is_moving:
            return

        home = self.get_parameter("home").value
        self.is_moving = True
        self.get_logger().info(f"HOME 이동 시작: {home}")

        try:
            time.sleep(0.5)
            self.moveit2.move_to_configuration(home)
            if self.wait_executed_with_timeout():
                self.get_logger().info("HOME 이동 완료")
            else:
                self.get_logger().error("HOME 이동 timeout")
        except Exception as e:
            self.get_logger().error(f"HOME 이동 예외: {e}")
        finally:
            self.is_moving = False

    def wait_executed_with_timeout(self, timeout_s=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.moveit2.wait_until_executed():
                return True
            time.sleep(0.1)
        return False

    # ----------------------------
    # 자동 Approach 각도 계산 (Joint 2, 3 보정)
    # ----------------------------
    def get_approach(self, angles):
        """
        바닥 각도를 기준으로 2번(Shoulder)과 3번(Elbow) 관절을 보정하여 
        수직 상단 대기 위치를 계산합니다.
        """
        approach = list(angles)
        approach[1] -= 0.25  # Shoulder를 뒤로 젖힘
        approach[2] += 0.15  # Elbow를 위로 올림
        return approach

    # ----------------------------
    # 그리퍼 제어
    # ----------------------------
    def control_gripper(self, open_mode=True):
        target = [0.01] if open_mode else [0.0]
        self.gripper.move_to_configuration(target)
        time.sleep(1.2)
    # ----------------------------
    # 안전 이동 보조 함수
    # ----------------------------
    # [추가] 이동 유닛 함수: 코드 중복을 줄이고 각 동작 사이 대기를 관리
    def move_to_and_wait(self, angles, label=""):
        time.sleep(1.0) # 계획 전 안정화 시간
        self.get_logger().info(f"{label} 이동 시도...")
        self.moveit2.move_to_configuration(angles)
        if self.wait_executed_with_timeout(10.0):
            self.get_logger().info(f"{label} 이동 완료")
            time.sleep(0.5) # 도착 후 잔진동 대기
            return True
        else:
            self.get_logger().error(f"{label} 이동 실패!")
            return False

    def wait_executed_with_timeout(self, timeout_s=20.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.moveit2.wait_until_executed():
                return True
            time.sleep(0.1)
        return False

    # ----------------------------
    # 컴퓨터 수 명령 처리
    # ----------------------------
    def computer_move_callback(self, msg: Int8):
        if not self.joint_states_ready: # 추가
            self.get_logger().warn("조인트 정보를 아직 받지 못했습니다. 잠시 후 다시 시도하세요.")
            return

        if not self.home_done or self.is_moving:
            return

        move_id = int(msg.data)
        if move_id < 0 or move_id > 8:
            return

        threading.Thread(
            target=self.execute_move_task,
            args=(move_id,),
            daemon=True
        ).start()

    # [수정] 메인 시나리오: 로딩존 -> 말 집기 -> 셀 이동 -> 말 놓기 -> 홈
    def execute_move_task(self, move_id: int):
        self.is_moving = True
        
        # 1) 필요한 각도 로드
        loading_zone = self.get_parameter("loading_zone").value
        loading_approach = self.get_approach(loading_zone)
        cell_angles = self.get_parameter(f"cell_{move_id}").value
        cell_approach = self.get_approach(cell_angles)
        home_angles = self.get_parameter("home").value

        try:
            # --- PHASE 1: 로딩 존에서 말 집기 ---
            self.control_gripper(open_mode=True)
            if not self.move_to_and_wait(loading_approach, "로딩존 Approach"): return
            if not self.move_to_and_wait(loading_zone, "로딩존 Ground"): return
            self.control_gripper(open_mode=False) # 말 집기
            if not self.move_to_and_wait(loading_approach, "로딩존 Retract"): return

            # --- PHASE 2: 셀 위치에 말 놓기 ---
            if not self.move_to_and_wait(cell_approach, f"Cell {move_id} Approach"): return
            if not self.move_to_and_wait(cell_angles, f"Cell {move_id} Ground"): return
            self.control_gripper(open_mode=True) # 말 놓기
            if not self.move_to_and_wait(cell_approach, f"Cell {move_id} Retract"): return

            # --- PHASE 3: 홈 위치로 복귀 ---
            self.move_to_and_wait(home_angles, "최종 Home 복귀")

        except Exception as e:
            self.get_logger().error(f"시퀀스 실행 중 예외 발생: {e}")
        finally:
            self.is_moving = False

    # --- 기존 함수 유지 (setup_obstacles, control_gripper 등) ---
    def control_gripper(self, open_mode=True):
        target = [0.019] if open_mode else [0.0]
        self.gripper.move_to_configuration(target)
        time.sleep(1.5) # 그리퍼 동작 시간 충분히 부여

    def wait_executed_with_timeout(self, timeout_s=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.moveit2.wait_until_executed():
                return True
            time.sleep(0.1)
        return False

    def heartbeat(self):
        while rclpy.ok():
            time.sleep(5.0)

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