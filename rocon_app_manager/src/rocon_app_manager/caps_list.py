#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
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
from capabilities import client
from capabilities import discovery
from capabilities import service_discovery
import capabilities.srv as capabilities_srvs
from .exceptions import MissingCapabilitiesException

##############################################################################
# Class
##############################################################################


class CapsList(object):
    '''
    CapsLists stores the data about the available capabilities retrieved from the capability server
    '''
    def __init__(self):
        '''
        Retrieve the specifications for the available interfaces and providers from the capability server

        @raise rospy.exceptions.ROSException: Exception is raised when retrieving the capability data
                                              from the capability server returned errors
                                              or when waiting for the capability server's service times out.
        '''
        self._default_timeout = 1.0

        self._caps_client = client.CapabilitiesClient("capability_server")
        self._spec_index, errors = service_discovery.spec_index_from_service("capability_server",
                                                                             self._default_timeout)
        if errors:
            raise rospy.exceptions.ROSException("Couldn't get specification index. Error: " + str(errors))

        self._available_interfaces = []
        self._available_semantic_interfaces = []
        self._providers = dict()
        for interface in self._spec_index.interfaces:
            if self._spec_index.specs[interface].default_provider:
                self._available_interfaces.append(interface)
                self._providers[interface] = self._spec_index.specs[self._spec_index.specs[interface].default_provider]
        for interface in self._spec_index.semantic_interfaces:
            if self._spec_index.specs[interface].default_provider:
                self._available_semantic_interfaces.append(interface)
                self._providers[interface] = self._spec_index.specs[self._spec_index.specs[interface].default_provider]

    def compatibility_check(self, app):
        '''
        Checks, if all required capabilities of an app are available

        @param app: app data of the app to be checked
        @type app: Rapp

        @raise MissingCapabilitiesException: Raised, if one or more required capabilities are not available.
        '''

        all_caps_available = True
        missing_capabilities = str()
        key = 'required_capabilities'
        if key in app.data:
            for cap in app.data[key]:
                if not cap["name"] in self._available_interfaces:
                    if not cap["name"] in self._available_semantic_interfaces:
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
        @type preferred_provider: string

        @return: true, if using the capability succeeded, false otherwise
        @type: boolean
        '''

        return self._caps_client.use_capability(name, preferred_provider)

    def stop_capability(self, name):
        '''
        Triggers the stop of the capability via the capability server

        @param name: name of the capability to stop
        @type name: string

        @return: true, if freeing the capability succeeded, false otherwise
        @type: boolean
        '''

        return self._caps_client.free_capability(name)

    def get_cap_remappings(self, cap, caps_remap_from_list, caps_remap_to_list):
        '''
        Gathers the required remappings for a specific capability

        The rapp description is expected to define all topics, services and actions required for the capability
        interfaces the rapp is depending on. This information is added to the 'caps_remap_from_list' list.

        Next the (semantic) capability's interface specification as well as the provider specification is parsed
        in order to determine the new topic, service and action names. Here three cases are possible:
         * if normal interface, remap to what is specified there
         * if semantic interface, remap to the semantic interface's remappings
         * if the provider specifies own remappings, apply them as well
        The final remapping is stored in 'caps_remap_to_list'.

        @param cap: cap data as specified in the rapp description
        @type name: dict

        @param caps_remap_from_list: topics to be remapped
        @type name: dict

        @param caps_remap_to_list: new names for remapped topics
        @type name: dict

        @raise MissingCapabilitiesException: The requested cap is not available.
        '''

        interface = None
        semantic_interface = False
        # check if cap is available, should not be needed, since rapp should have been pruned
        if cap["name"] in self._available_interfaces:
            interface = self._spec_index.interfaces[cap["name"]]
        else:
            if cap["name"] in self._available_semantic_interfaces:
                semantic_interface = True
                interface = self._spec_index.semantic_interfaces[cap["name"]]
            else:
                raise MissingCapabilitiesException(cap["name"])

        # gather all required (semantic) interface remappings or raise warnings if data is missing
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
                    rospy.logwarn("App Manager : Capability service '" + service
                                  + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this service.")
            for service in interface.provided_services:
                if service in cap['interface']['services']['provides']:
                    caps_remap_from_list.append(cap['interface']['services']['provides'][service])
                    caps_remap_to_list.append(service)
                else:
                    rospy.logwarn("App Manager : Capability service '" + service
                                  + "' not specified in rapp description."
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

        # check the interface's provider for additional remappings - not yet supported
        for remap in self._providers[interface.name].remappings:
            if remap in caps_remap_to_list:
                index = caps_remap_to_list.index(remap)
                caps_remap_to_list[index] = self._providers[interface.name].remappings[remap]
