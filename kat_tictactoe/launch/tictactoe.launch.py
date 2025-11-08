from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kat_tictactoe',   
            executable='game_manager', # setup.py/entry_points 기준 이름
            name='game_manager_node',
            output='screen'
        ),
        Node(
            package='kat_tictactoe',
            executable='player_interface',
            name='player_interface',
            output='screen'
        )
    ])

