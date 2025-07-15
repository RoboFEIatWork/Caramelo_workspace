import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'caramelo_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linux24-04',
    maintainer_email='victoroliveiraayres@gmail.com',
    description='Navigation package for Caramelo autonomous robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_waypoint_navigator = caramelo_navigation.simple_waypoint_navigator:main',
            'checkpoint_navigator = caramelo_navigation.checkpoint_navigator:main',
            'interactive_robot_positioner = caramelo_navigation.interactive_robot_positioner:main',
            'autonomous_waypoint_navigator = caramelo_navigation.autonomous_waypoint_navigator:main',
            'robocup_work_navigator = caramelo_navigation.robocup_work_navigator:main',
            'goalpose_mapping = caramelo_navigation.goalpose_mapping:main',
            'cmd_vel_safety_filter = caramelo_navigation.cmd_vel_safety_filter:main',
            'cmd_vel_monitor = caramelo_navigation.cmd_vel_monitor:main',
            'laser_scan_filter = caramelo_navigation.laser_scan_filter:main',
            'simple_goal_navigator = caramelo_navigation.simple_goal_navigator:main',
            'map_based_waypoint_navigator = caramelo_navigation.map_based_waypoint_navigator:main',
            'simple_waypoint_follower = caramelo_navigation.simple_waypoint_follower:main',
            'lifecycle_startup_helper = caramelo_navigation.lifecycle_startup_helper:main',
            'caramelo_waypoint_nav = caramelo_navigation.caramelo_waypoint_nav:main',
            'amcl_initializer = caramelo_navigation.amcl_initializer:main',
            'workstation_navigator = caramelo_navigation.workstation_navigator:main',
        ],
    },
)
