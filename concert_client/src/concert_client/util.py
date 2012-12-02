#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_client/LICENSE 
#
import roslib; roslib.load_manifest('concert_client')
import rospy
from gateway_msgs.srv import *
from gateway_msgs.msg import ConnectionType,Rule,RemoteRule

def createRemoteRule(gateway,rule):
    r = RemoteRule()
    r.gateway = gateway
    r.rule = rule
    return r

def createRule(name,type):
    r = Rule()
    r.name = name
    r.type = type
    r.node = ''
    return r
