#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from kat_msgs.msg import GameStart, BoardState, ComputerMove, GameResult
import random

class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager_node')

        # 0: Empty, 1: Player(X), -1: Computer(O)
        self.board = [0] * 9
        self.human_first = random.choice([True, False])
        self.current_turn = 1 if self.human_first else -1
        self.game_over = False

        # --- Publishers & Subscribers ---
        self.game_start_pub = self.create_publisher(GameStart, '/kat/game_start', 10)
        self.game_result_pub = self.create_publisher(GameResult, '/kat/game_result', 10)
        self.computer_move_pub = self.create_publisher(Int8, '/kat/computer_move', 10)
        self.ai_request_pub = self.create_publisher(BoardState, '/kat/ai_request', 10)

        self.create_subscription(Int8, '/kat/player_move', self.player_move_callback, 10)
        self.create_subscription(ComputerMove, '/kat/ai_response', self.ai_move_callback, 10)

        self.get_logger().info(f"게임 매니저 시작. 선공: {'플레이어' if self.human_first else '컴퓨터'}")

        # 게임 시작 신호 전송 (GUI가 뜰 시간을 주기 위해 약간 지연)
        self.start_timer = self.create_timer(1.0, self.publish_game_start)

    def publish_game_start(self):
        """게임 시작 메시지 발행"""
        msg = GameStart()
        msg.human_first = self.human_first
        msg.initial_board = self.board
        self.game_start_pub.publish(msg)
        self.get_logger().info(">>> 게임 시작 신호 발행됨 <<<")

        # 타이머 제거
        if self.start_timer:
            self.start_timer.cancel()
            self.start_timer = None

        # 컴퓨터 선공이면 첫 수 요청
        if not self.human_first:
            self.request_ai_move()

    def player_move_callback(self, msg):
        """플레이어의 수신 처리"""
        if self.game_over or self.current_turn != 1:
            return

        index = msg.data
        if 0 <= index < 9 and self.board[index] == 0:
            self.board[index] = 1 # 플레이어는 항상 1
            self.get_logger().info(f"플레이어 착수: {index}")
            
            if self.check_game_over(): return

            # 턴 변경 및 AI 요청
            self.current_turn = -1
            self.request_ai_move()

    def request_ai_move(self):
        """AI에게 현재 보드 상태를 보내 다음 수 요청"""
        self.get_logger().info("AI에게 수 요청 중...")
        msg = BoardState()
        msg.board = self.board
        self.ai_request_pub.publish(msg)

    def ai_move_callback(self, msg):
        """AI의 응답 처리"""
        if self.game_over or self.current_turn != -1:
            return

        move = msg.move
        if 0 <= move < 9 and self.board[move] == 0:
            self.board[move] = -1 # 컴퓨터는 항상 -1
            self.get_logger().info(f"AI 착수: {move}")

            # 로봇팔에게 이동 명령 전달
            robot_msg = Int8()
            robot_msg.data = move
            self.computer_move_pub.publish(robot_msg)

            if self.check_game_over(): return

            self.current_turn = 1
            self.get_logger().info("플레이어 턴입니다.")
        else:
            self.get_logger().error(f"AI가 잘못된 수를 두었습니다: {move}")

    def check_game_over(self):
        """승리 조건 검사 및 결과 발행"""
        wins = [(0,1,2),(3,4,5),(6,7,8),(0,3,6),(1,4,7),(2,5,8),(0,4,8),(2,4,6)]
        
        for i, j, k in wins:
            if self.board[i] == self.board[j] == self.board[k] != 0:
                winner = self.board[i]
                self.game_over = True
                
                res = GameResult()
                if winner == 1:
                    res.result = GameResult.RESULT_PLAYER_WINS
                    self.get_logger().info("게임 종료: 플레이어 승리!")
                else:
                    res.result = GameResult.RESULT_COMPUTER_WINS
                    self.get_logger().info("게임 종료: 컴퓨터 승리!")
                
                self.game_result_pub.publish(res)
                return True

        if 0 not in self.board:
            self.game_over = True
            res = GameResult()
            res.result = GameResult.RESULT_DRAW
            self.get_logger().info("게임 종료: 무승부!")
            self.game_result_pub.publish(res)
            return True

        return False

def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
