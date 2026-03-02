from setuptools import find_packages
from setuptools import setup

setup(
    name='ev_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('ev_msgs', 'ev_msgs.*')),
)
