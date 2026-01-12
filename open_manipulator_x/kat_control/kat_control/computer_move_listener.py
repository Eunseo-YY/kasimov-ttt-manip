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
            # 1) 메쉬 로드
            mesh = trimesh.load(stl_path)

            # trimesh가 Scene으로 읽어오는 경우가 있어서 안전 처리
            if hasattr(mesh, "dump"):
                mesh = mesh.dump(concatenate=True)

            mesh.apply_scale(0.001)

            # 2) CollisionObject 만들기
            obj = CollisionObject()
            obj.id = "board"
            obj.operation = CollisionObject.ADD

            # 중요: MoveIt이 쓰는 프레임으로 맞추는 게 안전함 (너 출력에 link1이었음)
            obj.header.frame_id = "world"
            obj.header.stamp = rclpy.time.Time().to_msg()

            # 3) Mesh 메시지 조립
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

            # 4) Pose (meshes 개수랑 mesh_poses 개수 같아야 함)
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0
            
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 1.0
            pose.orientation.w = 0.0
            obj.mesh_poses.append(pose)

            # 중요: 1번만 쏘면 씹힐 수 있어서 여러 번 publish
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
        if not self.joint_states_ready:
            self.joint_states_ready = True
            self.get_logger().info("joint_states 첫 수신")

            if not self.obstacle_added:
                self.setup_obstacles()

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
    def get_approach_configuration(self, target_angles):
        """
        바닥 각도를 기준으로 2번(Shoulder)과 3번(Elbow) 관절을 보정하여 
        수직 상단 대기 위치를 계산합니다.
        """
        approach = list(target_angles)
        approach[1] -= 0.25  # Shoulder를 뒤로 젖힘
        approach[2] += 0.15  # Elbow를 위로 올림
        return approach
    

    # ----------------------------
    # 그리퍼 제어
    # ----------------------------
    def control_gripper(self, open_mode=True):
        target = [0.019] if open_mode else [0.0]
        self.gripper.move_to_configuration(target)
        time.sleep(1.0)

    # ----------------------------
    # 컴퓨터 수 명령 처리
    # ----------------------------
    def computer_move_callback(self, msg: Int8):
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

    def execute_move_task(self, move_id: int):
        self.is_moving = True
        key = f"cell_{move_id}"

        # 파라미터에서 목표 각도 가져오기
        target_angles = self.get_parameter(key).value

        # 기본 포즈들
        pick = self.get_parameter("pick").value  
        home = self.get_parameter("home").value  

        # 접근/이탈 포즈(셀 위 대기)
        approach_angles = self.get_approach_configuration(target_angles)

        # 공통 실행 헬퍼(코드 중복 줄이기)
        def go(name: str, q, planning_time=10.0, timeout_s=12.0, close_gripper=None):
            self.moveit2.planning_time = float(planning_time)
            self.get_logger().info(f"{name} 이동 시도 | planning_time={planning_time} | q={q}")

            self.moveit2.move_to_configuration(list(q))

            ok = self.wait_executed_with_timeout(timeout_s=timeout_s)
            if ok:
                self.get_logger().info(f"{name} 이동 성공")
                if close_gripper is not None:
                    # close_gripper=True면 닫기, False면 열기
                    self.control_gripper(open_mode=(not close_gripper))
            else:
                self.get_logger().error(f"{name} 이동 실패/timeout")
            return ok

        try:
            # 0) 시작 전 그리퍼 열기
            self.control_gripper(open_mode=True)

            # 1) pick으로 가서 집기(닫기)
            if not go("PICK", pick, planning_time=100.0, timeout_s=15.0):
                return
            self.control_gripper(open_mode=False)  

            # 2) home으로 복귀(집은 상태 유지)
            if not go("HOME(after pick)", home, planning_time=100.0, timeout_s=15.0):
                return

            # 3) 셀 위 상단 대기(approach)
            if not go(f"APPROACH({key})", approach_angles, planning_time=100.0, timeout_s=20.0):
                return

            # 4) 실제 셀 위치(place)로 내려가기
            if not go(f"PLACE({key})", target_angles, planning_time=100.0, timeout_s=25.0):
                return

            # 5) 놓기(그리퍼 열기)
            self.control_gripper(open_mode=True)

            # 6) 다시 셀 위 상단 대기(retreat)
            if not go(f"RETREAT({key})", approach_angles, planning_time=100.0, timeout_s=20.0):
                return

            # 7) home으로 복귀
            go("HOME(final)", home, planning_time=100.0, timeout_s=15.0)

        except Exception as e:
            self.get_logger().error(f"이동 예외: {e}")
        finally:
            self.is_moving = False

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