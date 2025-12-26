from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kat_tictactoe',
            executable='game_manager',
            name='game_manager',
            output='screen',
        ),
        Node(
            package='kat_tictactoe',
            executable='player_interface',
            name='player_interface',
            output='screen',
        ),
        Node(
            package='kat_tictactoe',
            executable='tictactoe_logic',
            name='tictactoe_logic',
            output='screen',
        ),
    ])