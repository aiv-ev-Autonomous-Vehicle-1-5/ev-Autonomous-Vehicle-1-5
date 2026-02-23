from setuptools import find_packages
from setuptools import setup

setup(
    name='track_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('track_msgs', 'track_msgs.*')),
)
