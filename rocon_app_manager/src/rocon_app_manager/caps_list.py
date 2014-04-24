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

        :raises NotFoundException: Exception is raised when retrieving the capability data
                                   from the capability server returned errors
                                   or when waiting for the capability server's services times out
                                   or when failing to retrieve the capability server's nodelet manager name
        '''

        self._default_timeout = 3.0
        capability_server_name = rospy.get_param("~capability_server_name", "capability_server")

        # look for fully resolved name of the capability server
        self._cap_server_name = str()
        try:
            self._cap_server_name = rocon_python_comms.find_node(capability_server_name, unique=True)[0]
        except rocon_python_comms.NotFoundException as e:
            raise NotFoundException("Couldn't find capability server node. Error: " + str(e))

        # set up client
        self._caps_client = client.CapabilitiesClient(self._cap_server_name)
        if not self._caps_client.wait_for_services(self._default_timeout):
            raise NotFoundException("Timed out when waiting for the capability server (" + self._cap_server_name + ").")
        # establish_bond
        self._caps_client.establish_bond(self._default_timeout)

        # get the name of the capability server's nodelet manager
        service_name = self._cap_server_name + '/get_nodelet_manager_name'
        cap_server_nodelet_manager_srv = rospy.ServiceProxy(service_name, capabilities_srvs.GetNodeletManagerName)
        try:
            resp = cap_server_nodelet_manager_srv()
        except rospy.ServiceException as exc:
            raise NotFoundException("Couldn't not get capability server's nodelet manager name: " + str(exc))
        self.nodelet_manager_name = resp.nodelet_manager_name

        # get spec index
        try:
            self._spec_index, errors = service_discovery.spec_index_from_service(self._cap_server_name,
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

        :param app: app data of the app to be checked
        :type app: Rapp
        :raises MissingCapabilitiesException: One or more required capabilities are not available.
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

        :param name: name of the capability to start
        :type name: string
        :param preferred_provider: name of the preferred provider of the capability (optional)
        :type preferred_provider: string
        :returns: true, if using the capability succeeded, false otherwise
        :rtype: boolean
        '''

        return self._caps_client.use_capability(name, preferred_provider, self._default_timeout)

    def stop_capability(self, name):
        '''
        Triggers the stop of the capability via the capability server

        :param name: name of the capability to stop
        :type name: string
        :returns: true, if freeing the capability succeeded, false otherwise
        :rtype: boolean
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

        :param cap: cap data as specified in the rapp description
        :type name: dict
        :param caps_remap_from_list: topics to be remapped
        :type name: dict
        :param caps_remap_to_list: new names for remapped topics
        :type name: dict
        :raises MissingCapabilitiesException: The requested capability is not available.
        :raises rospy.ServiceException: Failed to retrieve provider remappings from capability server.
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

        # get provider remappings
        provider_remappings = dict()
        service_name = self._cap_server_name + '/get_remappings'
        cap_server_get_remap_srv = rospy.ServiceProxy(service_name, capabilities_srvs.GetRemappings)
        try:
            provider_remappings = cap_server_get_remap_srv(cap["name"])
        except rospy.ServiceException as exc:
            raise rospy.ServiceException("Couldn't not get provider remappings for interface '" + cap["name"]
                                         + "'. Error: " + str(exc))

        # gather all required (semantic) interface remappings or raise warnings if data is missing
        if not semantic_interface:
            for topic in interface.required_topics:
                if topic in cap['interface']['topics']['requires']:
                    caps_remap_from_list.append(cap['interface']['topics']['requires'][topic])
                    for remapped_topic in provider_remappings.topics:
                        if topic == remapped_topic.key:
                            topic = remapped_topic.value
                            rospy.logdebug("App Manager : Replaced interface topic '" + remapped_topic.key\
                                           + "' with provider remapping '" + topic)
                    caps_remap_to_list.append(topic)
                else:
                    rospy.logwarn("Rapp Manager : Capability topic '" + topic + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this topic.")
            for topic in interface.provided_topics:
                if topic in cap['interface']['topics']['provides']:
                    caps_remap_from_list.append(cap['interface']['topics']['provides'][topic])
                    for remapped_topic in provider_remappings.topics:
                        if topic == remapped_topic.key:
                            topic = remapped_topic.value
                            rospy.logdebug("App Manager : Replaced interface topic '" + remapped_topic.key\
                                           + "' with provider remapping '" + topic)
                    caps_remap_to_list.append(topic)
                else:
                    rospy.logwarn("Rapp Manager : Capability topic '" + topic + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this topic.")

            for service in interface.required_services:
                if service in cap['interface']['services']['requires']:
                    caps_remap_from_list.append(cap['interface']['services']['requires'][service])
                    for remapped_service in provider_remappings.services:
                        if service == remapped_service.key:
                            service = remapped_service.value
                            rospy.logdebug("App Manager : Replaced interface service '" + remapped_service.key\
                                           + "' with provider remapping '" + service)
                    caps_remap_to_list.append(service)
                else:
                    rospy.logwarn("Rapp Manager : Capability service '" + service
                                  + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this service.")
            for service in interface.provided_services:
                if service in cap['interface']['services']['provides']:
                    caps_remap_from_list.append(cap['interface']['services']['provides'][service])
                    for remapped_service in provider_remappings.services:
                        if service == remapped_service.key:
                            service = remapped_service.value
                            rospy.logdebug("App Manager : Replaced interface service '" + remapped_service.key\
                                           + "' with provider remapping '" + service)
                    caps_remap_to_list.append(service)
                else:
                    rospy.logwarn("Rapp Manager : Capability service '" + service
                                  + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this service.")

            for action in interface.required_actions:
                if action in cap['interface']['actions']['requires']:
                    caps_remap_from_list.append(cap['interface']['actions']['requires'][action])
                    for remapped_action in provider_remappings.actions:
                        if action == remapped_action.key:
                            action = remapped_action.value
                            rospy.logdebug("App Manager : Replaced interface action '" + remapped_action.key\
                                           + "' with provider remapping '" + action)
                    caps_remap_to_list.append(action)
                else:
                    rospy.logwarn("Rapp Manager : Capability action '" + action + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this action.")
            for action in interface.provided_actions:
                if action in cap['interface']['actions']['provides']:
                    caps_remap_from_list.append(cap['interface']['actions']['provides'][action])
                    for remapped_action in provider_remappings.actions:
                        if action == remapped_action.key:
                            action = remapped_action.value
                            rospy.logdebug("App Manager : Replaced interface action '" + remapped_action.key\
                                           + "' with provider remapping '" + action)
                    caps_remap_to_list.append(action)
                else:
                    rospy.logwarn("Rapp Manager : Capability action '" + action + "' not specified in rapp description."
                                  + " Can't apply automatic remapping for this action.")
        else:
            for remap in interface.remappings:
                remap_found = False
                semantic_remap = interface.remappings[remap]
                if 'topics' in cap['interface']:
                    if semantic_remap in cap['interface']['topics']['requires']:
                        caps_remap_from_list.append(cap['interface']['topics']['requires'][semantic_remap])
                        for remapped_topic in provider_remappings.topics:
                            if semantic_remap == remapped_topic.key:
                                semantic_remap = remapped_topic.value
                                rospy.logdebug("App Manager : Replaced interface topic '" + remapped_topic.key\
                                               + "' with provider remapping '" + semantic_remap)
                        caps_remap_to_list.append(semantic_remap)
                        remap_found = True
                if 'services' in cap['interface']:
                    if semantic_remap in cap['interface']['services']['requires']:
                        caps_remap_from_list.append(cap['interface']['services']['requires'][semantic_remap])
                        for remapped_service in provider_remappings.services:
                            if semantic_remap == remapped_service.key:
                                semantic_remap = remapped_service.value
                                rospy.logdebug("App Manager : Replaced interface service '" + remapped_service.key\
                                               + "' with provider remapping '" + semantic_remap)
                        caps_remap_to_list.append(semantic_remap)
                        remap_found = True
                if 'actions' in cap['interface']:
                    if semantic_remap in cap['interface']['actions']['requires']:
                        caps_remap_from_list.append(cap['interface']['actions']['requires'][semantic_remap])
                        for remapped_action in provider_remappings.actions:
                            if semantic_remap == remapped_action.key:
                                semantic_remap = remapped_action.value
                                rospy.logdebug("App Manager : Replaced interface action '" + remapped_action.key\
                                               + "' with provider remapping '" + semantic_remap)
                        caps_remap_to_list.append(semantic_remap)
                        remap_found = True
                if not remap_found:
                    rospy.logwarn("Rapp Manager : Semantic capability remapping '" + semantic_remap
                                  + "' not specified in rapp description. Can't apply automatic remapping for it.")

        # check the interface's provider for additional remappings - not yet supported
        for remap in self._providers[interface.name].remappings:
            if remap in caps_remap_to_list:
                index = caps_remap_to_list.index(remap)
                caps_remap_to_list[index] = self._providers[interface.name].remappings[remap]


#################################################
# Utilities
#################################################
def start_capabilities_from_caps_list(capabilities, caps_list):
    '''
      Starts up all required capaibilities

      :param capaibilities: list of starting capabilities
      :type: list of capabilities
      :param caps_list: capability list
      :type: CapsList
      :returns: True if successful. False with reason if it fails
      :rtype: boolean, string
    '''
    for cap in capabilities:
        try:
            start_resp = caps_list.start_capability(cap["name"])
        except rospy.ROSException as exc:
            message = ("Rapp Manager : service for starting capabilities is not available."
                       + " Will not start app. Error:"
                       + str(exc))
            rospy.logerr("Rapp Manager : %s" % message)
            return False, message
        except IOError as exc:
            message = ("Rapp Manager : error occurred while processing 'start_capability' service."
                       + " Will not start app. Error: "
                       + str(exc))
            rospy.logerr("Rapp Manager : %s" % message)
            return False, message
        if start_resp:
            rospy.loginfo("Rapp Manager : started required capability '" + str(cap["name"]) + "'.")
        else:
            message = ("Rapp Manager : starting capability '" + str(cap["name"]) + " was not successful."
                       " Will not start app.")
            rospy.logerr("Rapp Manager : %s" % message)
            return False, message
    rospy.loginfo("Rapp Manager : all required capabilities have been started.")
    return True, ""


def stop_capabilities_from_caps_list(capabilities, caps_list):
    '''
      Starts up all required capaibilities

      :param capaibilities: list of starting capabilities
      :type: list of capabilities
      :param caps_list: capability list
      :type: CapsList
      :returns: True if successful. False with reason if it fails
      :rtype: boolean, string
    '''
    for cap in capabilities:
        try:
            start_resp = caps_list.stop_capability(cap["name"])
        except rospy.ROSException as exc:
            message = ("Rapp Manager : Service for stopping capabilities is not available."
                       + " Error:" + str(exc))
            rospy.logerr("Rapp Manager : %s" % message)
            return False, message
        except IOError as exc:
            message = ("Rapp Manager : error occurred while processing 'stop_capability' service."
                       + " Error: " + str(exc))
            rospy.logerr("Rapp Manager : %s" % message)
            return False, message
        if start_resp:
            rospy.loginfo("Rapp Manager : Stopped required capability '" + str(cap["name"]) + "'.")
        else:
            message = ("Rapp Manager : stopping capability '" + str(cap["name"])
                       + " was not successful.")
            rospy.logerr("Rapp Manager : %s" % message)
            return False, message
    rospy.loginfo("Rapp Manager : All required capabilities have been stopped.")
    return True, ""
