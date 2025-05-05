from setuptools import setup
import os
from glob import glob

package_name = 'chall_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Asegura que los launch files sean instalados
        ('share/' + package_name + '/config', glob('config/*.yaml'))  # Asegura que los YAML sean instalados
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felipe',
    maintainer_email='felipe@todo.todo',
    description='Controlador',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ctrl = chall_2.ctrl:main',
        ],
    },
)
