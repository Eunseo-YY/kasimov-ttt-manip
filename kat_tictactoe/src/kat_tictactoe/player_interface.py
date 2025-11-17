#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
import tkinter as tk
from tkinter import messagebox
import json

class PlayerInterface(Node):
    def __init__(self):
        super().__init__('player_interface')

        # 상태
        self.game_over = False
        self.board = [0] * 9   # 0: 빈칸, 1: 플레이어(O), -1: 로봇(X)
        self.current_turn = 1  # GameManager에서 동기화 예정

        # 통신
        self.create_subscription(String, '/kat/game_start',
                                 self.game_start_callback, 10)
        self.player_move_pub = self.create_publisher(Int8, '/kat/player_move', 10)
        self.create_subscription(Int8, '/kat/computer_move',
                                 self.robot_move_callback, 10)
        self.create_subscription(String, '/kat/game_result',
                                 self.game_result_callback, 10)

        # GUI
        self.root = tk.Tk()
        self.root.title("사람 vs 로봇 틱택토")
        self.buttons = []
        for i in range(9):
            btn = tk.Button(self.root, text="", font=("Arial", 24),
                            width=5, height=2,
                            command=lambda i=i: self.human_move(i))
            btn.grid(row=i//3, column=i%3)
            self.buttons.append(btn)

    # ---- 콜백 ----
    def game_start_callback(self, msg: String):
        data = json.loads(msg.data)
        self.board = list(data.get("board", [0]*9))
        self.current_turn = 1 if data.get("human_first", True) else -1
        self.update_buttons()
        if self.current_turn == 1:
            messagebox.showinfo("게임 시작", "플레이어가 먼저 시작합니다!")
            self.get_logger().info("플레이어 선공.")
        else:
            messagebox.showinfo("게임 시작", "로봇이 먼저 시작합니다!")
            self.get_logger().info("로봇 선공.")

    def game_result_callback(self, msg: String):
        if self.game_over:
            return
        result = msg.data
        self.get_logger().info(f"게임 결과 수신: {result}")
        self.game_over = True
        messagebox.showinfo("게임 종료", result)
        for btn in self.buttons:
            btn.config(state=tk.DISABLED)

    def human_move(self, idx: int):
        if self.game_over or self.current_turn != 1:
            return
        if not (0 <= idx < 9) or self.board[idx] != 0:
            return
        self.board[idx] = 1
        self.update_buttons()
        self.get_logger().info(f"플레이어가 {idx}번 칸에 수.")
        self.current_turn = -1

        msg = Int8()
        msg.data = int(idx)
        self.player_move_pub.publish(msg)

    def robot_move_callback(self, msg: Int8):
        idx = int(msg.data)
        if self.game_over:
            return
        if 0 <= idx < 9 and self.board[idx] == 0:
            self.board[idx] = -1
            self.update_buttons()
            self.get_logger().info(f"로봇이 {idx}번 칸에 수.")
            self.current_turn = 1

    # ---- GUI ----
    def update_buttons(self):
        mapping = {0: "", 1: "O", -1: "X"}
        for i, val in enumerate(self.board):
            self.buttons[i].config(text=mapping[val])
            if self.game_over:
                self.buttons[i].config(state=tk.DISABLED)

    def spin(self):
        def pump():
            rclpy.spin_once(self, timeout_sec=0.05)
            self.root.after(50, pump)
        pump()
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = PlayerInterface()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from kat_msgs.msg import GameStart, GameResult
import tkinter as tk
from tkinter import messagebox
import threading

class PlayerInterface(Node):
    def __init__(self, gui_root):
        super().__init__('player_interface')
        self.gui_root = gui_root
        
        self.board_buttons = []
        self.game_over = True # 시작 신호를 받기 전까지는 True
        self.my_turn = False

        # ROS 통신
        self.create_subscription(GameStart, '/kat/game_start', self.on_game_start, 10)
        self.create_subscription(Int8, '/kat/computer_move', self.on_computer_move, 10)
        self.create_subscription(GameResult, '/kat/game_result', self.on_game_result, 10)
        self.move_pub = self.create_publisher(Int8, '/kat/player_move', 10)

        self.setup_gui()

    def setup_gui(self):
        self.gui_root.title("TicTacToe ROS 2 Player")
        self.gui_root.geometry("300x350")
        
        self.status_label = tk.Label(self.gui_root, text="게임 대기 중...", font=('Arial', 14))
        self.status_label.pack(pady=10)

        frame = tk.Frame(self.gui_root)
        frame.pack()

        for i in range(9):
            btn = tk.Button(frame, text="", font=('Arial', 24), width=5, height=2,
                            command=lambda idx=i: self.on_button_click(idx))
            btn.grid(row=i//3, column=i%3)
            self.board_buttons.append(btn)
            
    def on_game_start(self, msg):
        self.game_over = False
        self.my_turn = msg.human_first
        
        # 보드 초기화
        for i, val in enumerate(msg.initial_board):
            text = ""
            if val == 1: text = "X"
            elif val == -1: text = "O"
            self.board_buttons[i].config(text=text, state=tk.NORMAL)

        self.update_status()
        self.get_logger().info("GUI: 게임 시작!")

    def on_computer_move(self, msg):
        idx = msg.data
        if 0 <= idx < 9:
            self.board_buttons[idx].config(text="O", disabledforeground="blue")
            self.my_turn = True
            self.update_status()

    def on_button_click(self, idx):
        if not self.my_turn or self.game_over: return
        if self.board_buttons[idx]['text'] != "": return

        # 내 움직임 표시
        self.board_buttons[idx].config(text="X", disabledforeground="red")
        self.my_turn = False
        self.update_status()

        # 매니저에게 전송
        msg = Int8()
        msg.data = idx
        self.move_pub.publish(msg)

    def on_game_result(self, msg):
        self.game_over = True
        self.my_turn = False
        self.update_status()
        
        res_text = {
            GameResult.RESULT_PLAYER_WINS: "승리했습니다! 🎉",
            GameResult.RESULT_COMPUTER_WINS: "패배했습니다.. 🤖",
            GameResult.RESULT_DRAW: "무승부입니다. 🤝"
        }.get(msg.result, "게임 종료")
        
        messagebox.showinfo("게임 결과", res_text)

    def update_status(self):
        if self.game_over:
            txt = "게임 종료"
        else:
            txt = "당신의 턴 (X)" if self.my_turn else "컴퓨터 생각 중... (O)"
        self.status_label.config(text=txt)

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    
    player_node = PlayerInterface(root)
    
    # ROS 2 스피닝을 별도 스레드에서 실행 (GUI 멈춤 방지)
    spinner = threading.Thread(target=ros_spin_thread, args=(player_node,), daemon=True)
    spinner.start()

    try:
        root.mainloop()
    finally:
        if rclpy.ok():
            player_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
