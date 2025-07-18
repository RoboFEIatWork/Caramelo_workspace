import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'caramelo_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='work',
    maintainer_email='robofei.atwork@gmail.com',
    description='Task execution package for Caramelo autonomous robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_executor = caramelo_tasks.task_executor_node:main',
            'task_navigator = caramelo_tasks.task_navigator:main',
        ],
    },
)
