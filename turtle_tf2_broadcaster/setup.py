from setuptools import setup

package_name = 'turtle_tf2_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felipe',
    maintainer_email='felipe@todo.todo',
    description='Paquete para transformar la posición de turtle a tf2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_pose_transform = turtle_tf2_broadcaster.turtle_pose_transform:main',
        ],
    },
)
