#!/usr/bin/env python3
import os
import platform
import sys
import warnings
from distutils.core import setup, Extension
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['maze'],
    package_dir={'': 'src'}
)
setup(**d)
