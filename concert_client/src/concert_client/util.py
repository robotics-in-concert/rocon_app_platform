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
import gateway_msgs.msg as gateway_msgs


def createRemoteRule(gateway, rule):
    r = gateway_msgs.RemoteRule()
    r.gateway = gateway
    r.rule = rule
    return r


def createRule(name, type):
    r = gateway_msgs.Rule()
    r.name = name
    r.type = type
    r.node = ''
    return r
