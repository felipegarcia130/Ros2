from setuptools import find_packages, setup

package_name = 'signal_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/signal_processing']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/signal_processing_launch.py']),  # Línea agregada
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felipe',
    maintainer_email='felipe@todo.todo',
    description='Nodo generador y procesador de señales en ROS2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_generator = signal_processing.signal_generator:main',
            'process = signal_processing.process:main',
        ],
    },
)
