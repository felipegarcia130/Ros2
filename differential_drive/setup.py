from setuptools import setup
from glob import glob
import os

package_name = 'differential_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Esta línea es clave
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felipe',
    maintainer_email='felipe@todo.todo',
    description= 'Nodo para manejar un robot de conducción diferencial',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_drive_node = differential_drive.differential_drive_node:main',
            'differential_drive_controller = differential_drive.differential_drive_controller:main',
        ],
    },
)
