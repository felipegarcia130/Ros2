from setuptools import find_packages, setup

package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/puzzlebot_sim']),
        ('share/puzzlebot_sim', ['package.xml']),
        ('share/puzzlebot_sim/launch', ['launch/puzzlebot.launch.py']),
        ('share/puzzlebot_sim/urdf', ['urdf/puzzlebot.urdf']),
        ('share/puzzlebot_sim/meshes', [
            'meshes/chassis.stl',
            'meshes/wheel.stl',
            'meshes/camera.stl',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felipe',
    maintainer_email='felipe@todo.todo',
    description='Simulador de robot Puzzlebot',
    license='Apache License 2.0',
    # tests_require=['pytest'],  # ← comentario o eliminación
    entry_points={
        'console_scripts': [
            'puzzlebot_simulator = puzzlebot_sim.puzzlebot_simulator:main',
        ],
    },
)
