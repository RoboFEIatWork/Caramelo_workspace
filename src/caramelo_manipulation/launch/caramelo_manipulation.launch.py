from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                '/home/linux24-04/Caramelo_workspace/venv310/bin/python3',
                '/home/linux24-04/Caramelo_workspace/src/caramelo_manipulation/caramelo_manipulation/caramelo_manip.py'
            ],
            shell=False,
            output='screen'
        )
    ])
    