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
# Functions
##############################################################################

def parse_arguments():
    default_applist_directory = rocon_appmanager.get_default_applist_directory()
    parser = argparse.ArgumentParser(description='Robot application manager')
    parser.add_argument('-i', '--inventory', action='store',
                   default=default_applist_directory,
                   help='directory to search for application lists [%s]' % default_applist_directory)
    args = parser.parse_args()
    print args.inventory
    return args

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

  rospy.init_node('appmanager')
  args = parse_arguments()
  manager = rocon_appmanager.AppManager()
  manager.spin()
