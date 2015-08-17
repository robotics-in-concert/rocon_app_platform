#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-py/rocon_app_platform/license/LICENSE
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
    rospy.init_node('rapp_manager_script')
    standalone_rapp_manager = rocon_app_manager.Standalone()
    standalone_rapp_manager.spin()
