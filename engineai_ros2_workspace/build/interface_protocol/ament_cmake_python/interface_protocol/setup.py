from setuptools import find_packages
from setuptools import setup

setup(
    name='interface_protocol',
    version='0.0.1',
    packages=find_packages(
        include=('interface_protocol', 'interface_protocol.*')),
)
