#!/usr/bin/env python2

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup_args = generate_distutils_setup(
     packages=['py_uav_path_planning'],
     package_dir={'': 'src'},
     install_requires=requirements
)

setup(**setup_args)
