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
from concert_client.concertclient import *

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

    rospy.init_node('concert_client')
    client = ConcertClient()
    rospy.loginfo('Concert Client : initialised')
    client.spin()
