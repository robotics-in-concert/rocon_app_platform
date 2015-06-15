#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import copy
import os
import subprocess
import tempfile

import rospy
import traceback
import rocon_python_utils
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_std_msgs.msg as rocon_std_msgs
from .exceptions import MissingCapabilitiesException

from . import utils

##############################################################################
# Class
##############################################################################


class Rapp(object):
    '''
        Got many inspiration and imported from willow_app_manager implementation
    '''
    # standard args that can be put inside a rapp launcher, the rapp manager
    # will fill these args in when starting the rapp

    def __init__(self, rapp_specification):
        '''
           :param package: this rapp is nested in
           :type package: :py:class:`catkin_pkg.package.Package`
           :param package_relative_rapp_filename: string specified by the package export
           :type package_relative_rapp_filename: os.path
        '''
        self._connections = _create_empty_connection_type_dictionary()
        self._raw_data = rapp_specification
        self.data = rapp_specification.data
        self.data['status'] = 'Ready'
        self.data['published_interfaces'] = []
        self.data['published_parameters'] = {}
        self.data['implementations'] = []

    def __repr__(self):
        string = ''
        for d in self.data:
            string += d + ' : ' + str(self.data[d]) + '\n'
        return string

    def to_msg(self):
        '''
          Converts this app definition to ros msg format.

          :returns: ros message format of Rapp
          :rtype: rocon_app_manager_msgs.Rapp
        '''
        a = rapp_manager_msgs.Rapp()
        a.name = self.data['name']
        a.display_name = self.data['display_name']
        a.description = self.data['description']
        a.compatibility = self.data['compatibility']
        a.status = self.data['status']
        a.icon = rocon_python_utils.ros.icon_to_msg(self.data['icon'])
        a.implementations = []

        key = 'public_interface'
        if key in self.data:
            a.public_interface = [rocon_std_msgs.KeyValue(key, str(val)) for key, val in self.data[key].items()]

        key = 'public_parameters'
        if key in self.data:
            a.public_parameters = [rocon_std_msgs.KeyValue(str(key), str(val)) for key, val in self.data[key].items()]

        return a

    def install(self, dependency_checker):
        '''
          Installs all dependencies of the specified rapp

          :param dependency_checker: DependencyChecker object for installation of the rapp dependencies
          :type dependency_checker: :py:class:`rocon_app_utilities.rapp_repositories.DependencyChecker`

          :returns: A C{tuple} of a flag for the installation success and a string containing the reason of failure
          :rtype: C{tuple}
        '''
        success = False

        # Trigger the installation of all rapp dependencies
        rapps = []
        rapps.append(self.data['name'])
        try:
            dependency_checker.install_rapp_dependencies(rapps)
        except Exception as e:
            return success, str(e)

        # Update the rospack cache
        devnull = open(os.devnull, 'w')
        subprocess.call(['rospack', 'profile'], stdout=devnull, stderr=subprocess.STDOUT)
        devnull = devnull.close()

        success = True

        return success, str()

    def published_interfaces_to_msg_list(self):
        '''
        Convert the published interfaces as a rocon_app_manager_msgs.PublishedInterface list.
        '''
        return self.data['published_interfaces']

    def published_parameters_to_msg_list(self):
        '''
        Convert the published parameters as a rocon_std_msgs.KeyValue list.
        '''
        return utils.dict_to_KeyValue(self.data['published_parameters'])

    def start(self, application_namespace, gateway_name, rocon_uri_string, remappings=[], parameters=[], force_screen=False, simulation=False,
              caps_list=None):
        '''
          Some important jobs here.

          1) run the rapp launcher under the unique robot name namespace

          This guarantees that flipped entities generate unique node id's that won't collide when communicating
          with each other (refer to https://github.com/robotics-in-concert/rocon_multimaster/issues/136).

          2) Apply remapping rules while ignoring the namespace underneath.

          :param application_namespace: unique name granted indirectly via the gateways, we namespace everything under this
          :type application_namespace: str
          :param gateway_name: unique name granted to the gateway
          :type gateway_name: str
          :param rocon_uri_string: uri of the app manager's platform (used as a check for compatibility)
          :type rocon_uri_string: str - a rocon uri string
          :param remapping: rules for the app flips.
          :type remapping: list of rocon_std_msgs.msg.Remapping values.
          :param parameters: requested public_parameters
          :type parameters: list of rocon_std_msgs.msg.KeyValue
          :param force_screen: whether to roslaunch the app with --screen or not
          :type force_screen: boolean
          :param simulation: whether the rapp manager is for simulated robot or not
          :type simulation: boolean
          :param caps_list: this holds the list of available capabilities, if app needs capabilities
          :type caps_list: CapsList
        '''
        data = self.data

        try:
            capability_nodelet_manager_name = caps_list.nodelet_manager_name if caps_list else None

            published_parameters = utils.apply_requested_public_parameters(data['public_parameters'], parameters)

            temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
            self._launch = utils.prepare_launcher(data, published_parameters, application_namespace, gateway_name, rocon_uri_string, capability_nodelet_manager_name, force_screen, simulation, temp)

            # Better logic for the future, 1) get remap rules from capabilities. 2) get remap rules from requets. 3) apply them all. It would be clearer to understand the logic and easily upgradable
            if 'required_capabilities' in data:  # apply capability-specific remappings needed
                utils.apply_remapping_rules_from_capabilities(self._launch, data, caps_list)

            self._connections, published_interfaces = utils.apply_remapping_rules_from_start_app_request(self._launch, data, remappings, application_namespace)

            utils.resolve_chain_remappings(self._launch.config.nodes)
            self._launch.start()

            data['status'] = 'Running'
            data['published_parameters'] = published_parameters
            data['published_interfaces'] = published_interfaces

            connections = copy.deepcopy(self._connections)
            return True, "Success", connections
            #return True, "Success", self._connections['subscribers'], self._connections['publishers'], \
            #    self._connections['services'], self._connections['action_clients'], self._connections['action_servers']

        except rospy.ServiceException as e:
            rospy.logerr("App Manager : Couldn't get cap remappings. Error: " + str(e))
            return False, "Error while launching " + data['name'], _create_empty_connection_type_dictionary()
        except MissingCapabilitiesException as e:
            rospy.logerr("Rapp Manager : couldn't get capability remappings. Error: " + str(e))
            return False, "Error while launching " + data['name'], _create_empty_connection_type_dictionary()
        except Exception as e:
            traceback.print_stack()
            rospy.logerr("Rapp Manager : error while launching " + data['launch'] + " : " + str(e))
            data['status'] = "Error while launching " + data['launch']
            return False, "Error while launching " + data['name'], _create_empty_connection_type_dictionary()
        finally:
            os.unlink(temp.name)

    def stop(self):
        data = self.data
        connections = copy.deepcopy(self._connections)

        try:
            if self._launch:
                try:
                    self._launch.shutdown()
                finally:
                    self._launch = None
                    data['status'] = 'Ready'
                    data['published_parameters'] = {}
                    data['published_interfaces'] = []
                rospy.loginfo("Rapp Manager : stopped rapp [%s]" % data['name'] + "'.")
        except Exception as e:
            print str(e)
            error_msg = "Error while stopping rapp '" + data['name'] + "'."
            rospy.loginfo(error_msg)
            data['status'] = 'Error'
            return False, error_msg, connections

        return True, "Success", connections

    def is_running(self):
        '''
         Is the rapp both launched and currently running?

         Actually three possible states 1) not launched 2) running, 3) stopped
         Could acutally return a tertiary value, but rapp manager doesn't need
         to make any decision making about that (for now), so just return
         running or not.

         Used by the rapp_manager.

         :returns: True if the rapp is executing or False otherwise.
         :rtype: Bool
        '''
        if not self._launch:
            return False
        elif self._launch.pm and self._launch.pm.done:
            # time.sleep(1.0)  # do we need this sleep?
            return False
        return True


##############################################################################
# Utilities
##############################################################################
def convert_rapps_from_rapp_specs(rapp_specs):
    '''
      Converts rocon_app_utilities.Rapp into rocon_app_manager.Rapp

      :param rapp_specs: dict of rapp specification
      :type rapp_specs: {ancestor_name: rocon_app_utilities.Rapp}

      :returns: runnable rapps
      :rtype: {ancestor_name:rocon_app_manager.Rapp}
    '''
    runnable_rapps = {}

    for name, spec in rapp_specs.items():
        r = Rapp(spec)
        r.data['ancestor_name'] = spec.ancestor_name
        runnable_rapps[name] = r

    return runnable_rapps

def _create_empty_connection_type_dictionary():
    '''
      Initialise connections which use as public interface

      :returns: dict of connections with empty list
      :rtype: {connection_type: list}
    '''
    PUBLIC_CONNECTION_TYPES = ['publishers', 'subscribers', 'services', 'action_clients', 'action_servers']
    connections = {}
    for connection_type in PUBLIC_CONNECTION_TYPES:
        connections[connection_type] = []
    return connections

