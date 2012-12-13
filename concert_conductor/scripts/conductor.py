#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_conductor/LICENSE
#
import roslib
roslib.load_manifest('concert_conductor')
import rospy
import concert_conductor

if __name__ == '__main__':

    rospy.init_node('conductor')
    conductor = concert_conductor.Conductor()
    conductor.spin()
