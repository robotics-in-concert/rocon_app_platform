#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_client/LICENSE
#
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('concert_client')
import rospy
import concert_client

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

    rospy.init_node('concert_client')
    client = concert_client.ConcertClient()
    client.spin()
