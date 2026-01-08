import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2
from geometry_msgs.msg import PoseStamped
import threading
import time
import os

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
        self.obstacle_added = False

        # ----------------------------
        # 1) 파라미터 선언
        # ----------------------------
        for i in range(9):
            self.declare_parameter(f"cell_{i}", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home", [0.0, 0.0, 0.0, 0.0])

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
        raw_path = "~/ws_kat/src/kasimov-ttt-manip/open_manipulator_x/kat_description/meshes/board/board_assembled.stl"
        stl_path = os.path.expanduser(raw_path)

        if not os.path.exists(stl_path):
            self.get_logger().error(f"STL 파일 없음: {stl_path}")
            return

        ps = PoseStamped()
        ps.header.frame_id = "joint0"
        ps.pose.position.x = 0.20
        ps.pose.position.y = 0.0
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0

        try:
            self.moveit2.add_collision_mesh(
                filepath=stl_path,
                id="board",
                pose=ps,
                scale=1.0
            )
            self.obstacle_added = True
            self.get_logger().info("보드 장애물 추가 완료")
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
        angles = self.get_parameter(key).value

        try:
            self.control_gripper(open_mode=True)
            self.moveit2.move_to_configuration(angles)

            if self.wait_executed_with_timeout():
                self.control_gripper(open_mode=False)
                self.get_logger().info(f"{key} 이동 성공")
            else:
                self.get_logger().error(f"{key} 이동 실패")
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
