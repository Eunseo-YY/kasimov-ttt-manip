#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
import random
import json
import tictactoe_logic

class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager_node')

        # 선후공 랜덤 결정
        self.human_first = random.choice([True, False])

        # 상태
        self.board = [0] * 9        # 0: 빈칸, 1: 사람, -1: 컴퓨터
        self.current_turn = 1 if self.human_first else -1
        self.game_over = False

        # 타이머 핸들(분리해서 관리)
        self.start_timer = None      # 시작 신호 1회 발행용
        self.ai_open_timer = None    # 컴퓨터 선공 시 첫 수 지연용
        self.ai_delay_timer = None   # 플레이어 수 이후 컴퓨터 수 지연용

        # 통신
        qos = 10
        self.move_sub = self.create_subscription(Int8, '/kat/player_move',
                                                 self.player_move_callback, qos)
        self.computer_move_pub = self.create_publisher(Int8, '/kat/computer_move', qos)
        self.game_result_pub = self.create_publisher(String, '/kat/game_result', qos)
        self.game_start_pub = self.create_publisher(String, '/kat/game_start', qos)

        self.get_logger().info("Game Manager 노드가 시작되었습니다.")
        self.print_initial_turn()

        # GUI가 구독 준비할 시간을 주고 시작 신호를 정확히 1회 발행
        self.start_timer = self.create_timer(0.5, self.publish_start_signal_once)

        # 컴퓨터 선공이면 첫 수를 1초 지연 후 둠
        if not self.human_first and not self.game_over:
            self.ai_open_timer = self.create_timer(1.0, self.initial_computer_turn_once)

    # ---- 타이머 유틸 ----
    def _stop_timer(self, timer_attr_name: str):
        """타이머를 안전하게 취소/파괴하고 속성을 None으로."""
        timer = getattr(self, timer_attr_name, None)
        if timer is not None:
            try:
                timer.cancel()
            except Exception:
                pass
            try:
                self.destroy_timer(timer)
            except Exception:
                pass
            setattr(self, timer_attr_name, None)

    # ---- 시작 신호 1회 발행 ----
    def publish_start_signal_once(self):
        start_data = {"human_first": self.human_first, "board": self.board}
        msg = String()
        msg.data = json.dumps(start_data)
        self.game_start_pub.publish(msg)
        self.get_logger().info(f"게임 시작 신호 발행: {msg.data}")
        # 원샷 처리
        self._stop_timer('start_timer')

    # ---- 컴퓨터 선공 첫 수 1회 실행 ----
    def initial_computer_turn_once(self):
        self._stop_timer('ai_open_timer')
        self.computer_turn()

    # ---- 로그 ----
    def print_initial_turn(self):
        if self.human_first:
            self.get_logger().info("플레이어가 먼저 시작합니다. 입력을 기다립니다...")
        else:
            self.get_logger().info("컴퓨터가 먼저 시작합니다.")

    # ---- 콜백 ----
    def player_move_callback(self, msg: Int8):
        if self.game_over:
            self.get_logger().warn("게임이 이미 종료되었습니다.")
            return
        if self.current_turn != 1:
            self.get_logger().warn("플레이어의 턴이 아닙니다.")
            return

        index = int(msg.data)
        if 0 <= index < 9 and self.board[index] == 0:
            self.get_logger().info(f"플레이어가 {index}번 칸에 수를 두었습니다.")
            self.board[index] = 1
            self.print_board()

            if self.check_winner():
                return

            self.current_turn = -1
            self.get_logger().info("컴퓨터의 턴입니다.")
            # 직전 지연 타이머가 남아 있으면 정리
            self._stop_timer('ai_delay_timer')
            # 0.5초 지연 후 컴퓨터 수
            self.ai_delay_timer = self.create_timer(0.5, self._computer_turn_once)
        else:
            self.get_logger().error(f"잘못된 입력입니다. {index}번 칸은 불가합니다.")

    def _computer_turn_once(self):
        self._stop_timer('ai_delay_timer')
        self.computer_turn()

    # ---- 메인 로직 ----
    def computer_turn(self):
        if self.game_over:
            return

        move = tictactoe_logic.get_best_move(self.board)
        if move == -1:
            self.get_logger().info("둘 곳이 없어 보류 → 무승부 확인.")
            self.check_winner()  # 무승부 여부 최종 확인
            return

        self.get_logger().info(f"컴퓨터가 {move}번 칸에 수를 두었습니다.")
        self.board[move] = -1

        move_msg = Int8()
        move_msg.data = move
        self.computer_move_pub.publish(move_msg)

        self.print_board()

        if self.check_winner():
            return

        self.current_turn = 1
        self.get_logger().info("플레이어의 턴입니다.")

    def check_winner(self):
        result_msg = String()
        winner_found = False

        if self.check_winner_logic(1):
            result_msg.data = "플레이어 승리!"
            winner_found = True
        elif self.check_winner_logic(-1):
            result_msg.data = "컴퓨터 승리!"
            winner_found = True
        elif 0 not in self.board:
            result_msg.data = "무승부!"
            winner_found = True

        if winner_found:
            self.get_logger().info(result_msg.data)
            self.game_result_pub.publish(result_msg)
            self.game_over = True
            # 남아 있을지 모르는 타이머들 정리
            self._stop_timer('ai_delay_timer')
            self._stop_timer('ai_open_timer')
            self._stop_timer('start_timer')
            return True
        return False

    @staticmethod
    def check_winner_logic(player):
        win_conditions = [
            [0,1,2],[3,4,5],[6,7,8],   # 가로
            [0,3,6],[1,4,7],[2,5,8],   # 세로
            [0,4,8],[2,4,6]            # 대각
        ]
        return any(all_value == player for all_value in
                   [ [player if idx in cond else None for idx in range(9)] for cond in win_conditions ]) or \
               any(all(board_val == player for board_val in [None]) for _ in [])

    def print_board(self):
        b = self.board
        m = {1: 'O', -1: 'X', 0: ' '}
        board_str = (
            "\n"
            f" {m[b[0]]} | {m[b[1]]} | {m[b[2]]} \n"
            "---+---+---\n"
            f" {m[b[3]]} | {m[b[4]]} | {m[b[5]]} \n"
            "---+---+---\n"
            f" {m[b[6]]} | {m[b[7]]} | {m[b[8]]} "
        )
        self.get_logger().info(board_str)

def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 전 타이머/노드 정리
        try:
            node._stop_timer('ai_delay_timer')
            node._stop_timer('ai_open_timer')
            node._stop_timer('start_timer')
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
