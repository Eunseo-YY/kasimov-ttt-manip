#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from kat_msgs.msg import BoardState, ComputerMove

class TicTacToeAI(Node):
    def __init__(self):
        super().__init__('tictactoe_ai_node')
        
        self.create_subscription(BoardState, '/kat/ai_request', self.calculate_move, 10)
        self.move_pub = self.create_publisher(ComputerMove, '/kat/ai_response', 10)
        self.get_logger().info('TicTacToe Minimax AI 대기 중...')

    def calculate_move(self, msg):
        """GameManager로부터 보드 상태를 받아 최적의 수를 계산"""
        current_board = list(msg.board)
        
        # 보드가 비어있으면(첫 수) 중앙이나 모서리를 선점하는 것이 효율적 (계산 속도 단축)
        if current_board.count(0) == 9:
             best_move = 4 # 중앙 선점
        else:
             best_move = self.find_best_move(current_board)

        self.get_logger().info(f'AI 결정 (Minimax): {best_move}')
        
        res = ComputerMove()
        res.move = best_move
        self.move_pub.publish(res)

    def evaluate(self, board):
        """보드의 현재 상태를 평가하여 점수를 반환"""
        # 승리 조합 확인
        win_conditions = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8], # 가로
            [0, 3, 6], [1, 4, 7], [2, 5, 8], # 세로
            [0, 4, 8], [2, 4, 6]             # 대각선
        ]
        
        for condition in win_conditions:
            if board[condition[0]] == board[condition[1]] == board[condition[2]]:
                if board[condition[0]] == -1: # 컴퓨터 승리
                    return 10
                elif board[condition[0]] == 1: # 플레이어 승리
                    return -10
        return 0 # 승자 없음

    def minimax(self, board, depth, is_maximizing):
        """미니맥스 재귀 알고리즘"""
        score = self.evaluate(board)

        # 기저 사례: 승패가 결정되었거나, 더 이상 둘 곳이 없는 경우
        if score == 10: return score - depth # 빨리 이길수록 더 높은 점수
        if score == -10: return score + depth # 늦게 질수록 더 높은 점수 (최대한 버팀)
        if 0 not in board: return 0 # 무승부

        if is_maximizing: # 컴퓨터 턴 (점수 최대화 시도)
            best = -1000
            for i in range(9):
                if board[i] == 0:
                    board[i] = -1
                    best = max(best, self.minimax(board, depth + 1, not is_maximizing))
                    board[i] = 0 # 백트래킹
            return best
        else: # 플레이어 턴 (컴퓨터 입장에서 점수 최소화 시도)
            best = 1000
            for i in range(9):
                if board[i] == 0:
                    board[i] = 1
                    best = min(best, self.minimax(board, depth + 1, not is_maximizing))
                    board[i] = 0 # 백트래킹
            return best

    def find_best_move(self, board):
        """가능한 모든 수에 대해 미니맥스를 수행하고 최적의 위치 반환"""
        best_val = -1000
        best_move = -1

        # 가능한 모든 빈칸을 탐색
        for i in range(9):
            if board[i] == 0:
                board[i] = -1 # 컴퓨터가 임시로 두어봄
                move_val = self.minimax(board, 0, False) # 다음은 플레이어 턴
                board[i] = 0 # 원상복구

                # 더 좋은 수를 발견하면 갱신
                if move_val > best_val:
                    best_move = i
                    best_val = move_val

        return best_move

def main(args=None):
    rclpy.init(args=args)
    node = TicTacToeAI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
