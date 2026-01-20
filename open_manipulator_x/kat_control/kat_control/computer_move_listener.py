import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Point, Pose
import threading
import time
import os
from ament_index_python.packages import get_package_share_directory
import trimesh
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject


class ComputerMoveListener(Node):
    def __init__(self):
        super().__init__("computer_move_listener")

        # 시뮬레이션 시간 사용 여부 확인
        try:
            sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        except Exception:
            sim_time = False
        self.get_logger().info(f"Sim Time 사용 여부: {sim_time}")

        # [추가] 시뮬레이션 오차 허용 범위 강제 설정 (Invalid Trajectory 에러 방지)
        os.system("ros2 param set /move_group trajectory_execution/allowed_start_tolerance 0.2")

        self.get_logger().info("=== 노드 시작 ===")

        # ----------------------------
        # 0) 내부 상태 플래그
        # ----------------------------
        self.is_moving = False
        self.joint_states_ready = False
        self.home_done = False
        self.obstacle_added = False

        # joint state 저장(디버깅/확장용)
        self.last_joint_state = None

        # ----------------------------
        # 1) 파라미터 선언
        # ----------------------------
        for i in range(9):
            self.declare_parameter(f"cell_{i}", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home", [0.0, 0.0, 0.0, 0.0])
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
    # 공통: MoveIt 실행 대기(중복 제거, 딱 1개만)
    # ----------------------------
    def wait_executed_with_timeout(self, timeout_s=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.moveit2.wait_until_executed():
                return True
            time.sleep(0.1)
        return False

    # ----------------------------
    # 장애물 추가 (joint_states 이후 실행)
    # ----------------------------
    def setup_obstacles(self):
        try:
            package_path = get_package_share_directory("kat_description")
            stl_path = os.path.join(package_path, "meshes", "board", "board_assembled.stl")
        except Exception:
            stl_path = os.path.expanduser(
                "~/ws_kat/src/kasimov-ttt-manip/open_manipulator_x/kat_description/meshes/board/board_assembled.stl"
            )

        if not os.path.exists(stl_path):
            self.get_logger().error(f"STL 파일 없음: {stl_path}")
            return

        try:
            mesh = trimesh.load(stl_path)

            # trimesh가 Scene으로 읽어오는 경우가 있어서 안전 처리
            if hasattr(mesh, "dump"):
                mesh = mesh.dump(concatenate=True)

            # mm -> m 스케일
            mesh.apply_scale(0.001)

            obj = CollisionObject()
            obj.id = "board"
            obj.operation = CollisionObject.ADD
            obj.header.frame_id = "world"
            obj.header.stamp = rclpy.time.Time().to_msg()

            mesh_msg = Mesh()

            for face in mesh.faces:
                tri = MeshTriangle()
                tri.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
                mesh_msg.triangles.append(tri)

            for v in mesh.vertices:
                pt = Point()
                pt.x = float(v[0])
                pt.y = float(v[1])
                pt.z = float(v[2])
                mesh_msg.vertices.append(pt)

            obj.meshes.append(mesh_msg)

            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 1.0
            pose.orientation.w = 0.0
            obj.mesh_poses.append(pose)

            # 여러 번 publish (씹힘 방지)
            pub = self.moveit2._MoveIt2__collision_object_publisher
            for _ in range(8):
                pub.publish(obj)
                time.sleep(0.15)

            self.obstacle_added = True
            self.get_logger().info("보드 장애물 추가 완료 (재전송 포함)")

        except Exception as e:
            self.get_logger().error(f"장애물 추가 실패: {e}")

    # ----------------------------
    # joint_states 콜백
    # ----------------------------
    def joint_states_cb(self, msg: JointState):
        self.last_joint_state = msg

        if not self.joint_states_ready:
            self.joint_states_ready = True
            self.get_logger().info("joint_states 첫 수신")

            if not self.obstacle_added:
                self.setup_obstacles()

        if self.joint_states_ready and not self.home_done:
            self.home_done = True
            threading.Thread(target=self.move_to_home_once, daemon=True).start()

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
            if self.wait_executed_with_timeout(10.0):
                self.get_logger().info("HOME 이동 완료")
            else:
                self.get_logger().error("HOME 이동 timeout")
        except Exception as e:
            self.get_logger().error(f"HOME 이동 예외: {e}")
        finally:
            self.is_moving = False

    # ----------------------------
    # 자동 Approach 각도 계산 (Joint 2, 3 보정)
    # ----------------------------
    def get_approach(self, angles):
        approach = list(angles)
        approach[1] -= 0.4  # Shoulder 뒤로
        approach[2] += 0.15  # Elbow 위로
        return approach

    # ----------------------------
    # 안전 이동 보조 함수
    # ----------------------------
    def move_to_and_wait(self, angles, label=""):
        time.sleep(1.0)  # 계획 전 안정화
        self.get_logger().info(f"{label} 이동 시도...")
        self.moveit2.move_to_configuration(angles)

        if self.wait_executed_with_timeout(10.0):
            self.get_logger().info(f"{label} 이동 완료")
            time.sleep(0.5)  # 도착 후 잔진동
            return True
        else:
            self.get_logger().error(f"{label} 이동 실패!")
            return False

    # ----------------------------
    # 컴퓨터 수 명령 처리
    # ----------------------------
    def computer_move_callback(self, msg: Int8):
        if not self.joint_states_ready:
            self.get_logger().warn("조인트 정보를 아직 받지 못했습니다. 잠시 후 다시 시도하세요.")
            return

        if not self.home_done or self.is_moving:
            return

        move_id = int(msg.data)
        if move_id < 0 or move_id > 8:
            return

        threading.Thread(target=self.execute_move_task, args=(move_id,), daemon=True).start()

    # ----------------------------
    # 메인 시나리오 (수정본)
    # ----------------------------
    def execute_move_task(self, move_id: int):
        self.is_moving = True
        self.get_logger().info(f"--- Task {move_id} 시작 ---")

        loading_zone = self.get_parameter("loading_zone").value
        loading_approach = [3.14159265, -0.43633, 0.0, 1.91986]
        cell_angles = self.get_parameter(f"cell_{move_id}").value
        cell_approach = self.get_approach(cell_angles)
        home_angles = self.get_parameter("home").value

        try:
            # PHASE 1: 로딩존
            self.control_gripper(open_mode=True)
            if not self.move_to_and_wait([3.14159265, 0.0, 0.0, 0.0], "반대 회전"): return
            time.sleep(1.0)  # 안정화 대기
            if not self.move_to_and_wait(loading_approach, "로딩존 Approach"): return
            if not self.move_to_and_wait([3.14159265, -1.13539816, 0.52359878, 1.57079633], "로딩존 Ground 1"): return
            if not self.move_to_and_wait(loading_zone, "로딩존 Ground 2"): return

            self.control_gripper(open_mode=False) # 말 집기 (여기서 확실히 닫혀야 함)
            
            # [중요] 집은 후 아주 약간의 대기 후 들어올리기
            time.sleep(0.2)
            if not self.move_to_and_wait(loading_approach, "로딩존 Retract"): return

            # PHASE 2: 중간 Home 복귀 (이동 경로 확보)
            # 0.0, 0.0... 대신 파라미터로 설정된 home_angles 사용 권장
            self.get_logger().info("안전 경로 확보를 위한 중간 Home 이동")
            if not self.move_to_and_wait(home_angles, "중간 Home"): return
            time.sleep(5.0) # 안정화 대기

            # PHASE 3: 셀 위치에 놓기
            if not self.move_to_and_wait(cell_approach, f"Cell {move_id} Approach"): return
            if not self.move_to_and_wait(cell_angles, f"Cell {move_id} Ground"): return

            self.control_gripper(open_mode=True) # 말 놓기
            
            time.sleep(0.7)
            if not self.move_to_and_wait(cell_approach, f"Cell {move_id} Retract"): return

            # PHASE 4: 최종 복귀
            self.move_to_and_wait(home_angles, "최종 Home 복귀")

        except Exception as e:
            self.get_logger().error(f"시퀀스 실행 중 예외 발생: {e}")
        finally:
            self.is_moving = False
            self.get_logger().info(f"--- Task {move_id} 종료 ---")

    # ----------------------------
    # 하트비트 (노드 생존 확인용)
    # ----------------------------

    def heartbeat(self):
        while rclpy.ok():
            # self.get_logger().info("Heartbeat...") # 너무 자주 뜨면 주석 처리하세요
            time.sleep(5.0)

    # ----------------------------
    # 그리퍼 제어 보강
    # ----------------------------
    def control_gripper(self, open_mode=True):
        label = "Open" if open_mode else "Close"
        target = [0.009] if open_mode else [0.002]
        
        self.get_logger().info(f"그리퍼 {label} 시도...")
        self.gripper.move_to_configuration(target)
        
        # 단순히 sleep하는 대신 gripper 인스턴스의 완료를 기다림
        t0 = time.time()
        success = False
        while time.time() - t0 < 3.0:  # 최대 3초 대기
            if self.gripper.wait_until_executed():
                success = True
                break
            time.sleep(0.1)
        
        if success:
            self.get_logger().info(f"그리퍼 {label} 완료")
        else:
            self.get_logger().warn(f"그리퍼 {label} 타임아웃 (강제 진행)")
        
        time.sleep(0.5) # 물리적 안정화 시간


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