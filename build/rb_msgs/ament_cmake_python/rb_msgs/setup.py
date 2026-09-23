from setuptools import find_packages
from setuptools import setup

setup(
    name='rb_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('rb_msgs', 'rb_msgs.*')),
)
