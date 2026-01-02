#!/usr/bin/env python3
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int8
from pymoveit2 import MoveIt2


class ComputerMoveListener(Node):
    def __init__(self):
        super().__init__("computer_move_listener")

        # =========================
        # 0) Callback group (Reentrant)
        #   - 같은 노드 안에서 subscription callback + param service 등 여러 콜백이
        #     동시에 처리될 수 있게 함 🙂 
        # =========================
        self.cb_group = ReentrantCallbackGroup()

        # =========================
        # 1) Parameters (default)
        #   - YAML 안 들어와도 기본값으로라도 움직일 수 있게 선언 🙂 
        # =========================
        for i in range(9):
            self.declare_parameter(f"cell_{i}", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("home", [0.0, 0.0, 0.0, 0.0])

        # =========================
        # 2) MoveIt2 settings
        #   - TF 결과 기반으로 link 이름 수정 완료 🙂 
        # =========================
        self.group_name = "arm"
        self.joint_names = ["joint1", "joint2", "joint3", "joint4"]

        # 중요: joint 이름이 아니라 "link" 이름을 넣어야 함 🙂 
        self.base_link_name = "link1"
        self.end_effector_name = "end_effector_link"

        self.moveit2: Optional[MoveIt2] = None

        try:
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=self.joint_names,
                base_link_name=self.base_link_name,
                end_effector_name=self.end_effector_name,
                group_name=self.group_name,
                # pymoveit2 버전에 따라 아래 인자가 없을 수도 있음 🙂 
                # callback_group=self.cb_group,
            )
            self.get_logger().info("MoveIt2 초기화 성공 🙂")
        except Exception as e:
            # MoveIt2 초기화 실패하면, 콜백에서 실행 못 하도록 None 유지 🙂 
            self.get_logger().error(f"MoveIt2 초기화 실패: {e}")

        # =========================
        # 3) Subscription
        # =========================
        self.create_subscription(
            Int8,
            "/kat/computer_move",
            self.computer_move_callback,
            10,
            callback_group=self.cb_group,
        )

        # 동시에 들어오는 명령이 겹치면 위험하니까 간단한 락으로 직렬화 🙂 
        self._move_lock = threading.Lock()

        self.get_logger().info("=== Computer Move Listener 가동 준비 완료 === 🙂")

    # -------------------------
    # Utility: 파라미터에서 joint 각도 리스트 읽기
    # -------------------------
    def _get_angles_from_param(self, key: str) -> Optional[List[float]]:
        try:
            angles = self.get_parameter(key).value
        except Exception as e:
            self.get_logger().error(f"파라미터 '{key}' 읽기 실패: {e}")
            return None

        if angles is None or len(angles) == 0:
            self.get_logger().error(f"파라미터 '{key}'가 비어있음. YAML/launch 로드 확인 필요 🙂")
            return None

        if len(angles) != len(self.joint_names):
            self.get_logger().error(
                f"'{key}' 값 개수({len(angles)}) != joint_names 개수({len(self.joint_names)}) 🙂"
            )
            return None

        # 타입 안정성: float로 강제 변환 (문자열/정수 섞여도 안전하게) 🙂 
        try:
            angles = [float(x) for x in angles]
        except Exception as e:
            self.get_logger().error(f"'{key}' 값이 숫자로 변환 불가: {angles} / err={e}")
            return None

        return angles

    # -------------------------
    # Subscription callback (절대 오래 붙잡지 않기)
    #   - 여기서는 "스레드 시작"까지만 하고 즉시 리턴 🙂 
    # -------------------------
    def computer_move_callback(self, msg: Int8):
        move = int(msg.data)

        if move < 0 or move > 8:
            self.get_logger().error(f"잘못된 위치 번호: {move} (0~8만 가능) 🙂")
            return

        if self.moveit2 is None:
            self.get_logger().error("MoveIt2가 초기화되지 않아서 실행 불가 🙂")
            return

        key = f"cell_{move}"
        angles = self._get_angles_from_param(key)
        if angles is None:
            return

        self.get_logger().info(f"이동 명령 수신: {key} -> {angles} 🙂")

        # 실행은 별도 스레드에서 처리 (콜백 블로킹 금지) 🙂 
        t = threading.Thread(target=self._do_move, args=(angles, key), daemon=True)
        t.start()

    # -------------------------
    # 실제 MoveIt 실행(블로킹 OK)
    # -------------------------
    def _do_move(self, angles: List[float], key: str):
        # 동시에 여러 명령 들어오면 로봇이 꼬일 수 있으니 락으로 직렬화 🙂 
        with self._move_lock:
            try:
                # move_to_configuration가 goal 전송 + 실행까지 포함하는 버전도 있고,
                # plan만 하는 버전도 있어서 wait_until_executed로 결과 기다림 🙂 
                self.get_logger().info(f"{key} 이동 시작 🙂")
                self.moveit2.move_to_configuration(angles)

                success = self.moveit2.wait_until_executed()

                if success:
                    self.get_logger().info(f"성공: {key} 지점 도착 🙂")
                else:
                    self.get_logger().error(f"실패: {key} 이동 실행 실패 (timeout/abort 가능) 🙂")

            except Exception as e:
                self.get_logger().error(f"{key} 동작 실행 중 예외: {e} 🙂")


def main(args=None):
    rclpy.init(args=args)
    node = ComputerMoveListener()

    # MultiThreadedExecutor가 핵심 🙂 
    # - subscription callback, parameter service, 내부 통신 등이 같이 돌아감 🙂 
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
