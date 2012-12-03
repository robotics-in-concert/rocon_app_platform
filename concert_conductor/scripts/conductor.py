#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_conductor/LICENSE 
#
import roslib; roslib.load_manifest('concert_conductor')
import rospy

from concert_conductor.concertconductor import *

if __name__ == '__main__' :
    
    rospy.init_node('concertconductor')
    conductor = ConcertConductor()
    rospy.loginfo('Initialized')
    conductor.spin()
    rospy.loginfo('Done')
