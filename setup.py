#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['metrics_refbox'],
    package_dir={'metrics_refbox': 'src/metrics_refbox'}
)

setup(**d)
