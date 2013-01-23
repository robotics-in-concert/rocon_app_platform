#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['concert_client'],
    package_dir={'': 'src'},
#    scripts=['scripts/gateway_info',
#             'scripts/remote_gateway_info'
#             ],
    requires=['redis', 'rospy', 'concert_msgs', 'appmanager_msgs', 'gateway_msgs', 'rocon_hub_client']
)

setup(**d)
