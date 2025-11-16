from setuptools import find_packages
from setuptools import setup

setup(
    name='hexapod_simulation',
    version='0.0.0',
    packages=find_packages(
        include=('hexapod_simulation', 'hexapod_simulation.*')),
)
