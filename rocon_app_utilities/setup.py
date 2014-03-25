#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_app_utilities'],
    package_dir={'': 'src'},
    scripts=['scripts/rapp'],
    requires=['roslib', 'rospy', 'rocon_utilities']
)

setup(**d)
