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
from capabilities import discovery
from capabilities import service_discovery
import capabilities.srv as capabilities_srvs
from .exceptions import MissingCapabilitiesException

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
        self.caps_specs = None
        self._get_capability_specifications()
        # maybe add check to make sure specifications for all procviders are available (necessary?)

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
            raise IOError("Service did not process request: " + str(exc))
        return

    def _get_capability_specifications(self):
        '''
        Retrieve the capability specifications from capability server 
        '''
        self.caps_specs, errors = service_discovery.spec_index_from_service("capability_server")
        if errors:
            raise IOError("Couldn't get capability specifications. Error: " + str(errors))

#        try:
#            rospy.wait_for_service('capability_server/get_capability_specs', 1.0)
#        except rospy.ROSException as exc:
#            raise IOError("Service timeout: " + str (exc))
#            return
#        cap_specs_srv = rospy.ServiceProxy('capability_server/get_capability_specs',
#                                                capabilities_srvs.GetCapabilitySpecs)
#        try:
#            resp_cap_specs = cap_specs_srv()
#            self.caps_specs = resp_cap_specs.capability_specs
#        except rospy.ServiceException as exc:
#            error = "Service did not process request: " + str(exc)
#            raise IOError(error)
#        return

    def compatibility_check(self, app):
        '''
        Checks if all required capabilities of an app are available
        '''
        all_caps_available = True
        missing_capabilities = str()
        key = 'required_capabilities'
        if key in app.data:
            for cap in app.data[key]:
                if not cap["name"] in self.available_caps:
                    if not cap["name"] in self.available_semantic_caps:
                        missing_capabilities = missing_capabilities + " " + cap["name"]
                        all_caps_available = False
        if not all_caps_available:
            raise MissingCapabilitiesException(missing_capabilities)

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

    def get_cap_remappings(self, cap, caps_remap_from_list, caps_remap_to_list):
        '''
        Gathers the required remappings for this app, based on the provided cap data
        
        @param cap: cap data as specified in the app description
        @type name: dict
        
        @raise MissingCapabilitiesException: The requested cap is not available.
        @raise Exception: Provided cap data is not sufficient.
        
        @return: remappings from the provided cap data
        @type: boolean
        '''
        interface = None
        semantic_interface = False
        # check if cap is available, should not be needed, since rapp would have been pruned
        if not cap["name"] in self.available_caps:
            semantic_interface = True
            if not cap["name"] in self.available_semantic_caps:
                raise MissingCapabilitiesException(cap["name"])

        if not semantic_interface:
            if cap["name"] in self.caps_specs.names:
                interface = self.caps_specs.interfaces[cap["name"]]
            else:
                print "error" # should never happen, since the cap server does not support caps without their specs. CHECK THAT!
                return
        else:
            if cap["name"] in self.caps_specs.names:
                interface = self.caps_specs.semantic_interfaces[cap["name"]]
            else:
                print "error" # should never happen, since the cap server does not support caps without their specs. CHECK THAT!
                return

        # gather all required remappings or raise warnings if data is missing
        if not semantic_interface:
            for topic in interface.required_topics:
                if topic in cap['interface']['topics']['requires']:
                    caps_remap_from_list.append(cap['interface']['topics']['requires'][topic])
                    caps_remap_to_list.append(topic)
                else:
                    rospy.logwarn("App Manager : Capability topic '" + topic + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this topic.")
            for topic in interface.provided_topics:
                if topic in cap['interface']['topics']['provides']:
                    caps_remap_from_list.append(cap['interface']['topics']['provides'][topic])
                    caps_remap_to_list.append(topic)
                else:
                    rospy.logwarn("App Manager : Capability topic '" + topic + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this topic.")

            for service in interface.required_services:
                if service in cap['interface']['services']['requires']:
                    caps_remap_from_list.append(cap['interface']['services']['requires'][service])
                    caps_remap_to_list.append(service)
                else:
                    rospy.logwarn("App Manager : Capability service '" + service + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this service.")
            for service in interface.provided_services:
                if service in cap['interface']['services']['provides']:
                    caps_remap_from_list.append(cap['interface']['services']['provides'][service])
                    caps_remap_to_list.append(service)
                else:
                    rospy.logwarn("App Manager : Capability service '" + service + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this service.")

            for action in interface.required_actions:
                if action in cap['interface']['actions']['requires']:
                    caps_remap_from_list.append(cap['interface']['actions']['requires'][action])
                    caps_remap_to_list.append(action)
                else:
                    rospy.logwarn("App Manager : Capability action '" + action + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this action.")
            for action in interface.provided_actions:
                if action in cap['interface']['actions']['provides']:
                    caps_remap_from_list.append(cap['interface']['actions']['provides'][action])
                    caps_remap_to_list.append(action)
                else:
                    rospy.logwarn("App Manager : Capability action '" + action + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this action.")
        else:
            for remap in interface.remappings:
                remap_found = False
                semantic_remap = interface.remappings[remap]
                if 'topics' in cap['interface']:
                    if semantic_remap in cap['interface']['topics']['requires']:
                        caps_remap_from_list.append(cap['interface']['topics']['requires'][semantic_remap])
                        caps_remap_to_list.append(semantic_remap)
                        remap_found = True
                if 'services' in cap['interface']:
                    if semantic_remap in cap['interface']['services']['requires']:
                        caps_remap_from_list.append(cap['interface']['services']['requires'][semantic_remap])
                        caps_remap_to_list.append(semantic_remap)
                        remap_found = True
                if 'actions' in cap['interface']:
                    if semantic_remap in cap['interface']['actions']['requires']:
                        caps_remap_from_list.append(cap['interface']['actions']['requires'][semantic_remap])
                        caps_remap_to_list.append(semantic_remap)
                        remap_found = True
                if not remap_found:
                    rospy.logwarn("App Manager : Semantic capability remapping '" + semantic_remap
                                  + "' not specified in rapp description. Can't apply automatic remapping for it.")
        return caps_remap_from_list, caps_remap_to_list
