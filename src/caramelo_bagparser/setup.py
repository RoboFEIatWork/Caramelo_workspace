from setuptools import setup

package_name = 'caramelo_bagparser'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', ['scripts/bag.sh']),
        ('share/' + package_name + '/matlab', ['matlab/process_bagfile.m']),
        ('share/' + package_name + '/launch', ['launch/parse_bag.launch.py']),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu@email.com',
    description='Parser de bag ROS para YAML via script MATLAB',
    license='MIT',
)
