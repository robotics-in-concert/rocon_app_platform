#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_appmanager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import argparse
import roslib
roslib.load_manifest('rocon_appmanager')
import rospy
import rocon_appmanager

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

  rospy.init_node('appmanager')
  manager = rocon_appmanager.AppManager()
  manager.spin()
