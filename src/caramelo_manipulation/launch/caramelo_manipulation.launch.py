from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Caminhos baseados no usuário atual
    venv_python = os.path.expanduser('~/Caramelo_workspace/venv310/bin/python3')
    script_path = os.path.expanduser('~/Caramelo_workspace/src/caramelo_manipulation/caramelo_manipulation/caramelo_manip.py')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[venv_python, script_path],
            shell=False,
            output='screen'
        )
    ])
