import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'caramelo_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor',
    maintainer_email='victoroliveiraayres@gmail.com',
    description='Pacote de bringup para o robô real Caramelo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Novos nós modulares (arquitetura atualizada)
            'encoder_reader_node = caramelo_bringup.encoder_reader_node:main',
            'odometry_fusion_node = caramelo_bringup.odometry_fusion_node:main',
            'pwm_controller_node = caramelo_bringup.pwm_controller_node:main',
            
            # Nós de utilidade mantidos
            'twist_converter_node = caramelo_bringup.twist_converter_node:main',
            'zed_imu_driver = caramelo_bringup.zed_imu_driver:main',
        ],
    },
)
