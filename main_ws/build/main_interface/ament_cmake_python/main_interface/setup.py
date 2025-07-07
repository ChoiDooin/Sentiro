from setuptools import find_packages
from setuptools import setup

setup(
    name='main_interface',
    version='0.0.0',
    packages=find_packages(
        include=('main_interface', 'main_interface.*')),
)
