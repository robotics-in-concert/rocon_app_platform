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
    Caps lists store the installed capabilities retrieved from the capability server
    '''
    def __init__(self):
        self.available_caps = []
        self.available_semantic_caps = []
        self._get_available_interfaces()
        self.caps_specs = []
        self._get_capability_specifications()

    def _get_available_interfaces(self):
        '''
        Retrieve available normal and semantic interfaces from capability server 
        '''
        try:
            rospy.wait_for_service('capability_server/get_interfaces', 1.0)
            rospy.wait_for_service('capability_server/get_semantic_interfaces', 1.0)
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

    def _get_capability_specifications(self):
        '''
        Retrieve the capability specifications from capability server 
        '''
        try:
            rospy.wait_for_service('capability_server/get_capability_specs', 1.0)
        except rospy.ROSException as exc:
            raise IOError("Service timeout: " + str (exc))
            return
        cap_specs_srv = rospy.ServiceProxy('capability_server/get_capability_specs',
                                                capabilities_srvs.GetCapabilitySpecs)
        try:
            resp_cap_specs = cap_specs_srv()
            self.caps_specs = resp_cap_specs.capability_specs
        except rospy.ServiceException as exc:
            error = "Service did not process request: " + str(exc)
            raise IOError(error)
        return

    def start_capability(self, name, preferred_provider=None):
        '''
        Triggers the start of the capability via the capability server
        
        @param name: name of the capability to start
        @type name: string
        
        @param preferred_provider: name of the preferred provider of the capability (optional)
        @type name: string
        
        @raise ROSException: raised if waiting for start_capability service times out
        @raise ServiceException: raised if an error occurs while the service is processed
        
        @return: true, if the service call for starting the capability returned true,
                 false if service call returned false or a timeout occurred while waiting for service
        @type: boolean
        '''
        try:
            rospy.wait_for_service('capability_server/start_capability', 1.0)
        except rospy.ROSException as exc:
            raise IOError("Service timeout: " + str (exc))
            return False
        start_cap_srv = rospy.ServiceProxy('capability_server/start_capability',
                                           capabilities_srvs.StartCapability)
        try:
            resp_start_cap = start_cap_srv(name, preferred_provider)
        except rospy.ServiceException as exc:
            error = "Service did not process request: " + str(exc)
            raise IOError(error)
        return resp_start_cap.successful

    def stop_capability(self, name):
        '''
        Triggers the stop of the capability via the capability server
        
        @param name: name of the capability to stop
        @type name: string
        
        @raise ROSException: raised if waiting for stop_capability service times out
        @raise ServiceException: raised if an error occurs while the service is processed
        
        @return: true, if the service call for stopping the capability returned true,
                 false if service call returned false or a timeout occurred while waiting for service
        @type: boolean
        '''
        try:
            rospy.wait_for_service('capability_server/stop_capability', 1.0)
        except rospy.ROSException as exc:
            raise IOError("Service timeout: " + str (exc))
            return False
        start_cap_srv = rospy.ServiceProxy('capability_server/stop_capability',
                                           capabilities_srvs.StopCapability)
        try:
            resp_start_cap = start_cap_srv(name)
        except rospy.ServiceException as exc:
            error = "Service did not process request: " + str(exc)
            raise IOError(error)
        return resp_start_cap.successful
