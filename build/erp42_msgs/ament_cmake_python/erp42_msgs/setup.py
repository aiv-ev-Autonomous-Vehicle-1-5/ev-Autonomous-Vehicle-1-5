from setuptools import find_packages
from setuptools import setup

setup(
    name='erp42_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('erp42_msgs', 'erp42_msgs.*')),
)
