#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mbot_autom_param_tuning_tools_ros', 'mbot_autom_param_tuning_tools'],
    package_dir={'mbot_autom_param_tuning_tools_ros': 'ros/src/mbot_autom_param_tuning_tools_ros',
                 'mbot_autom_param_tuning_tools': 'common/src/mbot_autom_param_tuning_tools'}
)

setup(**d)
