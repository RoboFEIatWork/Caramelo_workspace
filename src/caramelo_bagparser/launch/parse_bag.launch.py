import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for flexibility
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Path to the bag file to parse (optional)'
    )
    
    # Get the package share directory
    package_share_dir = get_package_share_directory('caramelo_bagparser')
    
    # Build script path using the package share directory
    script_path = os.path.join(package_share_dir, 'scripts', 'bag.sh')
    
    # Create the ExecuteProcess action to run the bag.sh script
    run_bag_script = ExecuteProcess(
        cmd=['bash', script_path, LaunchConfiguration('bag_file')],
        output='screen',
        shell=False,
        cwd=package_share_dir
    )

    return LaunchDescription([
        bag_file_arg,
        run_bag_script
    ])
