from setuptools import find_packages
from setuptools import setup

setup(
    name='wd_lift_robot',
    version='0.0.0',
    packages=find_packages(
        include=('wd_lift_robot', 'wd_lift_robot.*')),
)
