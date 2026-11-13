from setuptools import find_packages
from setuptools import setup

setup(
    name='autonom_bringup',
    version='0.1.0',
    packages=find_packages(
        include=('autonom_bringup', 'autonom_bringup.*')),
)
