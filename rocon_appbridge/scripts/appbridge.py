#!/usr/bin/env python       
import roslib; roslib.load_manifest('rocon_appbridge')

import rospy
from rocon_appbridge import *

if __name__ == '__main__':

    rospy.init_node('appbridge')
    bridge = AppBridge()
    rospy.loginfo("Initialized")
    bridge.spin()
    rospy.loginfo("Done")
