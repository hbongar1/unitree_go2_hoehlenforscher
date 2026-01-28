from setuptools import find_packages
from setuptools import setup

setup(
    name='go2_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('go2_msgs', 'go2_msgs.*')),
)
