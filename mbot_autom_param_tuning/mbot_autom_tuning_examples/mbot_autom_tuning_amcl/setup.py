#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mbot_autom_tuning_amcl_ros'],
    package_dir={'mbot_autom_tuning_amcl_ros': 'ros/src/mbot_autom_tuning_amcl_ros'}
)

setup(**d)
