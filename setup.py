#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['targets_path_planning', 'path_planning', 'gazebo_communicator', 'task_management'],
    package_dir={'': 'src'},
)

setup(**d)
