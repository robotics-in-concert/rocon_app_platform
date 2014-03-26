#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import yaml
import rospkg
import rospy
import traceback
import roslaunch.xmlloader
import rocon_python_utils
from roslaunch.config import load_config_default
from roslaunch.core import RLException
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_std_msgs.msg as rocon_std_msgs
from .exceptions import RappException, RappResourceNotExistException, RappMalformedException, MissingCapabilitiesException

# Local Imports
from .utils import *
from .pairing_client import PairingClient

##############################################################################
# Class
##############################################################################
class Rapp(object):
    '''
        Got many inspiration and imported from willow_app_manager implementation 
    '''
    # I should add a __slots__ definition here to make it easy to read

    # standard args that can be put inside a rapp launcher, the rapp manager
    # will fill these args in when starting the rapp
    standard_args = ['gateway_name', 'application_namespace', 'rocon_uri', 'capability_server_nodelet_manager_name']

    def __init__(self, rapp_specification, rospack):
        '''
           :param package: this rapp is nested in
           :type package: :py:class:`catkin_pkg.package.Package`
           :param package_relative_rapp_filename: string specified by the package export
           :type package_relative_rapp_filename: os.path
           :param rospack: utility cache to speed up ros resource searches
           :type rospack: :py:class:`rospkg.RosPack`
        '''
        self._connections = _init_connections()
        self._raw_data = rapp_specification

        try:
            self.data = load_specs_from_file(rapp_specification, rospack)
        except RappResourceNotExistException as e :
            reason = str(e) + ' [' + str(rapp_specification.ancestor_name) + ']'
            raise RappException(reason)
        except RappMalformedException as e: 
            reason = str(e) + ' [' + str(rapp_specification.ancestor_name) + ']'
            raise RappException(reason)
        self.data['status'] = 'Ready'

    def __repr__(self):
        string = ""
        for d in self.data:
            string += d + " : " + str(self.data[d]) + "\n"
        return string

    def to_msg(self):
        '''
          Converts this app definition to ros msg format.

          :returns: ros message format of Rapp
          :rtype: rocon_app_manager_msgs.msg.App
        '''
        a = rapp_manager_msgs.App()
        a.name = self.data['name']
        a.display_name = self.data['display_name']
        a.description = self.data['description']
        a.compatibility = self.data['compatibility']
        a.status = self.data['status']
        a.icon = rocon_python_utils.ros.icon_to_msg(self.data['icon'])
        a.pairing_clients = [pc.to_msg() for pc in self.data['pairing_clients']]

        key = 'required_capabilities'
        if key in self.data:
            a.required_capabilities = [cap['name'] for cap in self.data[key]]

        return a
          
    def start(self, application_namespace, gateway_name, rocon_uri_string, remappings=[], force_screen=False,
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
          :param force_screen: whether to roslaunch the app with --screen or not
          :type force_screen: boolean
          :param caps_list: this holds the list of available capabilities, if app needs capabilities
          :type caps_list: CapsList
        '''
        data = self.data

        try:
            nodelet_manager_name = caps_list.nodelet_manager_name if caps_list else None

            temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
            self._launch = prepare_launcher(data, application_namespace, gateway_name, rocon_uri_string, nodelet_manager_name, force_screen, temp)

            # Better logic for the future, 1) get remap rules from capabilities. 2) get remap rules from requets. 3) apply them all. It would be clearer to understand the logic and easily upgradable 
            if 'required_capabilities' in data: # apply capability-specific remappings needed
                apply_remapping_rules_from_capabilities(self._launch, data, caps_list)

            self._connections = apply_remapping_rules_from_start_app_request(self._launch, data, remappings, application_namespace)

            resolve_chain_remappings(self._launch.config.nodes)
            self._launch.start()

            data['status'] = 'Running'
            return True, "Success", self._connections['subscribers'], self._connections['publishers'], \
                self._connections['services'], self._connections['action_clients'], self._connections['action_servers']

        except MissingCapabilitiesException as e:
            rospy.logerr("App Manager : Couldn't get cap remappings. Error: " + str(e))
            return False, "Error while launching " + data['name'], [], [], [], [], []
        except Exception as e:
            print str(e)
            traceback.print_stack()
            rospy.logerr("App Manager : Error while launching " + data['launch'])
            data['status'] = "Error while launching " + data['launch']
            return False, "Error while launching " + data['name'], [], [], [], [], []
        finally:
            os.unlink(temp.name)

    def stop(self):
        data = self.data

        try:
            if self._launch:
                try:
                    self._launch.shutdown()
                finally:
                    self._launch = None
                    data['status'] = 'Ready'
                rospy.loginfo("App Manager : Stopped rapp [%s]" % data['name'] + "'.")
        except Exception as e:
            print str(e)
            error_msg = "Error while stopping rapp '" + data['name'] + "'."
            rospy.loginfo(error_msg)
            data['status'] = 'Error'
            return False, error_msg, self._connections['subscribers'], self._connections['publishers'], \
                self._connections['services'], self._connections['action_clients'], self._connections['action_servers']

        return True, "Success", self._connections['subscribers'], self._connections['publishers'], \
                self._connections['services'], self._connections['action_clients'], self._connections['action_servers']

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
def convert_rapps_from_rapp_specs(rapp_specs, rospack):
    '''
      Converts rocon_app_utilities.Rapp into rocon_app_manager.Rapp

      :param rapp_specs: dict of rapp specification
      :type rapp_specs: {ancestor_name: rocon_app_utilities.Rapp} 
      :param rospack: utility cache to speed up ros resource searches
      :type rospack: :py:class:`rospkg.RosPack`


      :returns: runnable rapps, defected rapps
      :rtype: {ancestor_name:rocon_app_manager.Rapp}, {ancestor_name: defect reason}
    '''
    runnable_rapps = {}
    defected_rapps = {}

    for name, spec in rapp_specs.items():
        try:
            r = Rapp(spec, rospack)
            runnable_rapps[name] = r
        except RappException as e:
            defected_rapps[name] = str(e)
    return runnable_rapps, defected_rapps


def _init_connections(): 
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


def load_specs_from_file(specification, rospack):
    '''
      Specification consists of resource which is file pointer. This function loads those files in memeory

      :param specification: Rapp Specification
      :type Specification: rocon_app_utilities.Rapp
      :param rospack: utility cache to speed up ros resource searches
      :type rospack: :py:class:`rospkg.RosPack`

      :returns Fully loaded rapp data dictionary
      :rtype: dict
    '''
    rapp_data = specification.data
    data = {}
    data['name'] = specification.ancestor_name
    data['display_name']      = rapp_data.get('display', data['name'])
    data['description']       = rapp_data.get('description', '')
    data['compatibility']     = rapp_data['compatibility']
    data['icon']              = _find_resource(rapp_data['icon'], rospack) if 'icon' in rapp_data else None
    data['launch']            = _find_resource(rapp_data['launch'], rospack)
    data['launch_args']       = _get_standard_args(data['launch'])
    data['public_interface']  = _load_public_interface(rapp_data.get('public_interface', None), rospack)

    data['public_parameters'] = _load_public_parameters(rapp_data.get('public_parameters',None), rospack) # TODO : It is not tested yet.
    data['pairing_clients']   = _load_pairing_clients(rapp_data.get('pairing_clients', []))

    required_capabilities = 'required_capabilities'
    if required_capabilities in rapp_data:
        data[required_capabilities] = [ c for c in rapp_data[required_capabilities]]

    return data

def _find_resource(resource, rospack):
    '''
      Find a rapp resource (.launch, .interface, icon) relative to the
      specified package path.

      :param rapp_name: name of the rapp, only used for log messages.
      :type rapp_name: str
      :param resource: a typical resource identifier to look for
      :type resource: pkg_name/file pair in str format.
      :param rospack: utility cache to speed up ros resource searches
      :type rospack: :py:class:`rospkg.RosPack`

      :raises: :exc:`.exceptions.RappResourcenotExistException` if the resource is not found
    '''
    try:
        return rocon_python_utils.ros.find_resource_from_string(resource, rospack)
    except rospkg.ResourceNotFound:
        raise RappResourceNotExistException("invalid rapp - %s does not exist" % (resource))


def _load_public_interface(public_interface_resource, rospack):
    '''
      loading public interfaces from file. If the given filepath

      :params public_interface_resource: absolute path of public interface file
      :type: str
      :param rospack: utility cache to speed up ros resource searches
      :type rospack: :py:class:`rospkg.RosPack`

      :returns: dict of public interface
      :rtype: {keys : []}

      :raises RappMalformedException: public interface contains mssing key
    '''
    d = {}
    keys = ['subscribers', 'publishers', 'services', 'action_clients', 'action_servers']

    # If public interface is not present. return empty list for each connection types
    if not public_interface_resource:
        d = { k : [] for k in keys}
        return d

    public_interface_file_path = _find_resource(public_interface_resource, rospack)
    with open(public_interface_file_path, 'r') as f:
        y = yaml.load(f.read())
        y = y or {}
        try:
            for k in keys:
                raw_data = y.get(k, [])

                new_data = []
                for r in raw_data:
                    #if r[0] == '/':  # originally removed these, but we really do need to reference such sometimes
                    #    r = r[1:len(r)]
                    new_data.append(r)
                d[k] = new_data

        except KeyError:
            raise RappMalformedException("Invalid interface, missing keys")
    return d


def _load_public_parameters(public_parameters_resource, rospack):
    '''
      :params public_parameters_resource: absolute path of public parameters file
      :type: str
      :param rospack: utility cache to speed up ros resource searches
      :type rospack: :py:class:`rospkg.RosPack`

      :returns: dict of public parameter 
      :rtype: {keys : []}
    '''

    # If public interface is not present. return empty list for each connection types
    if not public_parameters_resource:
        return {}

    d = {}
    public_parameters_file_path = find_resource(public_parameters_resource, rospack)

    # TODO: Do it!

    return d


def _load_pairing_clients(clients_data, appfile="UNKNOWN"):
    '''
      Load pairing client information from the .rapp file.
      :raises RappMalformedException: if the .rapp pairing clients definition was invalid.
    '''
    clients = []
    for c in clients_data:
        for reqd in ['type', 'manager']:
            if not reqd in c:
                raise RappMalformedException("malformed .rapp [%s], missing required key [%s]" % (appfile, reqd))
        client_type = c['type']
        manager_data = c['manager']
        if not type(manager_data) == dict:
            raise RappMalformedException("malformed .rapp [%s]: manager data must be a map" % (appfile))

        app_data = c.get('app', {})
        if not type(app_data) == dict:
            raise RappMalformedException("malformed appfile [%s]: app data must be a map" % (appfile))

        clients.append(PairingClient(client_type, manager_data, app_data))
    return clients

def _get_standard_args(roslaunch_file):
    '''
      Given the Rapp launch file, this function parses the top-level args
      in the file. Returns the complete list of top-level arguments that
      match standard args so that they can be passed to the launch file

      :param roslaunch_file: rapp launch file we are parsing for arguments
      :type roslaunch_file: str
      :returns: list of top-level arguments that match standard arguments. Empty
              list on parse failure
      :rtype: [str]
    '''
    try:
        loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False)
        unused_config = load_config_default([roslaunch_file], None, loader=loader,
                                     verbose=False, assign_machines=False)
        available_args = \
                [str(x) for x in loader.root_context.resolve_dict['arg']]
        return [x for x in available_args if x in Rapp.standard_args]
    except (RLException, rospkg.common.ResourceNotFound) as e:
        # The ResourceNotFound lets us catch errors when the launcher has invalid
        # references to resources
        rospy.logerr("App Manager : failed to parse top-level args from rapp " +
                     "launch file [" + str(e) + "]")
        return []
