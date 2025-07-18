from setuptools import setup
from glob import glob
import os

package_name = 'caramelo_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={package_name: package_name},
    package_data={
        package_name: ['data/*.json']
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu@email.com',
    description='Pacote de manipulação com visão computacional',
    license='MIT',
    tests_require=['pytest'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'manip_listener = caramelo_manipulation.manip_listener_launcher:main',
            'manip_listener_launcher = caramelo_manipulation.manip_listener_launcher:main',
            'manip_service = caramelo_manipulation.manip_service:main',
        ],
    },
)
