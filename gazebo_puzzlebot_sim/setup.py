from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Registro del paquete
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name
        ]),
        # package.xml
        ('share/' + package_name, [
            'package.xml'
        ]),
        # Archivos .launch
        ('share/' + package_name + '/launch', [
            'launch/sim_launch.py',
            'launch/gazebo_world_launch.py'
        ]),
        # Config (trayectoria)
        ('share/' + package_name + '/config', [
            'config/path.yaml'
        ]),
        # Modelos .sdf / .cfg
        ('share/' + package_name + '/models/puzzlebot', [
            'models/puzzlebot/model.sdf',
            'models/puzzlebot/model.cfg'
        ]),
        # Mundo .world
        ('share/' + package_name + '/worlds', [
            'worlds/puzzlebot_world.worlds'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felipe',
    maintainer_email='felipe@todo.todo',
    description='Simulación de trayectoria para el Puzzlebot en Gazebo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = gazebo_puzzlebot_sim.sim_node:main',
        ],
    },
)
