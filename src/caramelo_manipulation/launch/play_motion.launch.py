from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='caramelo_manipulation',
            executable='play_motion_node',
            name='play_motion_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'}, 
                {'baudrate': 1000000},
                {'log_file': '~/Caramelo_workspace/src/caramelo_manipulation/registro_movimentos.txt'}
            ]
        )
    ])

