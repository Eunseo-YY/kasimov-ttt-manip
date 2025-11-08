# -*- coding: utf-8 -*-
# board: 길이 9 리스트, 0=빈칸, 1=플레이어, -1=컴퓨터
import math

def check_winner(board):
    win = [
        [0,1,2],[3,4,5],[6,7,8],  # 가로
        [0,3,6],[1,4,7],[2,5,8],  # 세로
        [0,4,8],[2,4,6]           # 대각
    ]
    for c in win:
        if board[c[0]] != 0 and all(board[i] == board[c[0]] for i in c):
            return board[c[0]]     # 1=플레이어, -1=컴퓨터
    if 0 not in board:
        return 0                   # 무승부
    return None                    # 진행 중

def minimax(board, depth, is_maximizing):
    winner = check_winner(board)
    if winner is not None:
        if winner == -1:
            return 10 - depth
        elif winner == 1:
            return depth - 10
        else:
            return 0

    if is_maximizing:
        best = -math.inf
        for i in range(9):
            if board[i] == 0:
                board[i] = -1
                score = minimax(board, depth+1, False)
                board[i] = 0
                best = max(best, score)
        return best
    else:
        best = math.inf
        for i in range(9):
            if board[i] == 0:
                board[i] = 1
                score = minimax(board, depth+1, True)
                board[i] = 0
                best = min(best, score)
        return best

def get_best_move(board):
    best_score = -math.inf
    best_move = -1
    for i in range(9):
        if board[i] == 0:
            board[i] = -1
            score = minimax(board, 0, False)
            board[i] = 0
            if score > best_score:
                best_score = score
                best_move = i
    return best_move
