from setuptools import find_packages
from setuptools import setup

setup(
    name='entrance_detection_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('entrance_detection_msgs', 'entrance_detection_msgs.*')),
)
