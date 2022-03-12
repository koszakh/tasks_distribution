#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tasks_distribution', 'path_planning', 'gazebo_communicator', 'task_management', 'network_creation'],
    package_dir={'': 'src'},
)

setup(**d)
