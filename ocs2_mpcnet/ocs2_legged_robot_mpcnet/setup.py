#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(packages=["ocs2_legged_robot_mpcnet"], package_dir={"": "python"})

setup(**setup_args)
