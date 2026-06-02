from setuptools import find_packages
from setuptools import setup

setup(
    name='mi_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('mi_interfaces', 'mi_interfaces.*')),
)
