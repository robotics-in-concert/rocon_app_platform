#!/usr/bin/env python       
import roslib; roslib.load_manifest('rocon_kitchen')

import rospy
from rocon_kitchen.Kitchen import *

if __name__ == '__main__':

    rospy.init_node('kitchen')
    kitchen = Kitchen()
    rospy.loginfo("Initialized")
    kitchen.spin()
    rospy.loginfo("Done")
