#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_app_manager/LICENSE
#
##############################################################################
# Overview
##############################################################################
'''
 Capabilities handling for the app manager
'''
##############################################################################
# Imports
##############################################################################

import rospy
import capabilities.srv as capabilities_srvs

##############################################################################
# Class
##############################################################################

class CapsList(object):
    '''
     Caps lists store the installed capabilities retrieved from the capbility server
    '''
    def __init__(self):
        # set up a service client for normal and semantic interfaces
        try:
            rospy.wait_for_service('capability_server/get_interfaces', 2.0)
            rospy.wait_for_service('capability_server/get_semantic_interfaces', 2.0)
        except rospy.ROSException as exc:
            raise IOError("Service timeout: " + str (exc))
            return
        available_caps_srv = rospy.ServiceProxy('capability_server/get_interfaces', capabilities_srvs.GetInterfaces)
        available_semantics_caps_srv = rospy.ServiceProxy('capability_server/get_semantic_interfaces',
                                                          capabilities_srvs.GetSemanticInterfaces)
        try:
            # retrieve available capabilities 
            resp_caps = available_caps_srv()
            resp_semantic_caps = available_semantics_caps_srv()
            # store capabilities in list
            self.available_caps = resp_caps.interfaces
            self.available_semantic_caps = resp_semantic_caps.semantic_interfaces
        except rospy.ServiceException as exc:
            error = "Service did not process request: " + str(exc)
            raise IOError(error)
        return
