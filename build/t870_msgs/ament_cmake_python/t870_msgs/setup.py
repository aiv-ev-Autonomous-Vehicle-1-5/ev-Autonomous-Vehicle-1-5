from setuptools import find_packages
from setuptools import setup

setup(
    name='t870_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('t870_msgs', 't870_msgs.*')),
)
