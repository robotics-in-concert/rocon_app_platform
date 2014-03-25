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

from capabilities import client
from capabilities import discovery
from capabilities import service_discovery
import capabilities.srv as capabilities_srvs
import rocon_python_comms
import rospy
from .exceptions import MissingCapabilitiesException
from .exceptions import NotFoundException

##############################################################################
# Class
##############################################################################


class CapsList(object):
    '''
    CapsLists stores the data about the available capabilities retrieved from the capability server
    '''
    def __init__(self):
        '''
        Sets up a client for the capability server, including a bond.
        Also, retrieves the specifications for the available interfaces and providers from the capability server

        @raise rospy.exceptions.ROSException: Exception is raised when retrieving the capability data
                                              from the capability server returned errors
                                              or when waiting for the capability server's services times out
                                              or when failing to retrieve the capability server's nodelet manager name
        '''

        self._default_timeout = 3.0
        capability_server_node_name = rospy.get_param("~capability_server_name", "capability_server")

        # look for fully resolved name of the capability server
        try:
            fully_resolved_name = rocon_python_comms.find_node(capability_server_node_name, unique=True)[0]
        except rocon_python_comms.NotFoundException as e:
            raise NotFoundException("Couldn't find capability server node. Error: " + str(e))

        # set up client
        self._caps_client = client.CapabilitiesClient(fully_resolved_name)
        if not self._caps_client.wait_for_services(self._default_timeout):
            raise NotFoundException("Timed out when waiting for the capability server (" + fully_resolved_name + ").")
        # establish_bond
        self._caps_client.establish_bond(self._default_timeout)

        # get the name of the capability server's nodelet manager
        service_name = fully_resolved_name + '/get_nodelet_manager_name'
        cap_server_nodelet_manager_srv = rospy.ServiceProxy(service_name, capabilities_srvs.GetNodeletManagerName)
        try:
            resp = cap_server_nodelet_manager_srv()
        except rospy.ServiceException as exc:
            raise NotFoundException("Couldn't not get capability server's nodelet manager name: " + str(exc))
        self.nodelet_manager_name = resp.nodelet_manager_name

        # get spec index
        try:
            self._spec_index, errors = service_discovery.spec_index_from_service(fully_resolved_name,
                                                                                 self._default_timeout)
        except rospy.exceptions.ROSException as e:
            raise NotFoundException("Couldn't get specification index. Error: " + str(e))

        if errors:
            raise NotFoundException("Couldn't get specification index. Error: " + str(errors))

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

        return self._caps_client.use_capability(name, preferred_provider, self._default_timeout)

    def stop_capability(self, name):
        '''
        Triggers the stop of the capability via the capability server

        @param name: name of the capability to stop
        @type name: string

        @return: true, if freeing the capability succeeded, false otherwise
        @type: boolean
        '''

        return self._caps_client.free_capability(name, self._default_timeout)

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


#################################################
# Local Methods                
#################################################
def start_capabilities_for_rapp(capabilities, caps_list): 
    '''
      Starts up all required capaibilities

      :param capaibilities: list of starting capabilities
      :type: list of capabilities
      :param caps_list: capability list
      :type: CapsList

      :returns: True if successful. False with reason if it fails 
      :rtype: bool, str
    '''
    for cap in capabilities:
        try:
            start_resp = caps_list.start_capability(cap["name"])
        except rospy.ROSException as exc:
            message = ("App Manager : service for starting capabilities is not available."
                            + " Will not start app. Error:"
                            + str(exc))
            rospy.logerr("App Manager : %s" % message)
            return False, message
        except IOError as exc:
            message = ("App Manager : error occurred while processing 'start_capability' service."
                            + " Will not start app. Error: "
                            + str(exc))
            rospy.logerr("App Manager : %s" % message)
            return False, message
        if start_resp:
            rospy.loginfo("App Manager : started required capability '" + str(cap["name"]) + "'.")
        else:
            message = ("App Manager : starting capability '" + str(cap["name"]) + " was not successful."
                            " Will not start app.")
            rospy.logerr("App Manager : %s" % message)
            return False, message
    rospy.loginfo("App Manager : all required capabilities have been started.")
    return True, ""

def stop_capabilities_for_rapp(capabilities, caps_list):
    '''                                                     
      Starts up all required capaibilities
 
      :param capaibilities: list of starting capabilities
      :type: list of capabilities
      :param caps_list: capability list
      :type: CapsList
 
      :returns: True if successful. False with reason if it fails 
      :rtype: bool, str
    '''
    for cap in capabilities:
        try:
            start_resp = caps_list.stop_capability(cap["name"])
        except rospy.ROSException as exc:
            message = ("App Manager : Service for stopping capabilities is not available."
                            + " Error:" + str(exc))
            rospy.logerr("App Manager : %s" % message)
            return False, message
        except IOError as exc:
            message = ("App Manager : Error occurred while processing 'stop_capability' service."
                            + " Error: " + str(exc))
            rospy.logerr("App Manager : %s" % message)
            return False, message
        if start_resp:
            rospy.loginfo("App Manager : Stopped required capability '" + str(cap["name"]) + "'.")
        else:
            message = ("App Manager : Stopping capability '" + str(cap["name"])
                            + " was not successful.")
            rospy.logerr("App Manager : %s" % message)
            return False, message
    rospy.loginfo("App Manager : All required capabilities have been stopped.")
    return True, ""
