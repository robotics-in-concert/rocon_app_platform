#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_app_manager

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('rapp_manager')
    manager = rocon_app_manager.RappManager()
    manager.spin()
