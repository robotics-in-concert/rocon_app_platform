#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_conductor/LICENSE 
#

import roslib; roslib.load_manifest('concert_conductor')
import rospy


class ConcertConductor(object):

    def __init__(self):
        self.pub = {}
        self.pub['client_list'] = rospy.Publisher


    def spin(self):


    def log(self,msg):
        rospy.loginfo("Concert Conductor : " + msg)

