from setuptools import setup

package_name = 'caramelo_manipulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/play_motion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu@email.com',
    description='Pacote de manipulação do Caramelo que reproduz movimentos a partir de .txt',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'play_motion_node = caramelo_manipulation.play_motion_node:main',
            'manipulation_interface = caramelo_manipulation.manipulation_interface:main',
        ],
    },
)

