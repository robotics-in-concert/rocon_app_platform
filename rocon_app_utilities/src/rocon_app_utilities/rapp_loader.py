#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################
from .exceptions import InvalidRappException, RappResourceNotExistException, RappMalformedException
import yaml
import rospkg
import roslaunch.xmlloader
from roslaunch.config import load_config_default
from roslaunch.core import RLException
import rocon_python_utils
from rocon_console import console

def load_rapp_yaml_from_file(filename):
    '''
      Load rapp specs yaml from the given file

      :param filename: absolute path to the rapp definition
      :type filename: str

      :returns: dict of loaded rapp
      :rtype: dict

      :raises: InvalidRappFieldException: Rapp includes invalid filed
    '''
    RAPP_ATTRIBUTES = ['display', 'description', 'icon', 'public_interface', 'public_parameters', 'compatibility', 'launch', 'parent_name', 'pairing_clients', 'required_capabilities']

    with open(filename, 'r') as f:
        app_data = yaml.load(f.read())

        for d in app_data:
            if d not in RAPP_ATTRIBUTES:
                raise InvalidRappException('Invalid Field : [' + str(d) + '] Valid Fields : [' + str(RAPP_ATTRIBUTES) + ']')

    return app_data


def load_rapp_specs_from_file(specification, rospack=rospkg.RosPack()):
    '''
      Specification consists of resource which is file pointer. This function loads those files in memeory

      :param specification: Rapp Specification
      :type Specification: rocon_app_utilities.Rapp
      :param rospack: utility cache to speed up ros resource searches
      :type rospack: :py:class:`rospkg.RosPack`

      :returns Fully loaded rapp data dictionary
      :rtype: dict
    '''
    rapp_data = specification.raw_data
    data = {}
    data['name'] = specification.ancestor_name
    data['display_name']      = rapp_data.get('display', data['name'])
    data['description']       = rapp_data.get('description', '')
    data['compatibility']     = rapp_data['compatibility']
    data['icon']              = _find_resource(rapp_data['icon'], rospack) if 'icon' in rapp_data else None
    data['launch']            = _find_resource(rapp_data['launch'], rospack)
    data['launch_args']       = _get_standard_args(data['launch'])
    data['public_interface']  = _load_public_interface(rapp_data.get('public_interface', None), rospack)
    data['public_parameters'] = _load_public_parameters(rapp_data.get('public_parameters', None), rospack)  # TODO : It is not tested yet.

    if 'pairing_clients' in rapp_data:
        console.logwarn('Rapp Indexer : [%s] includes "pairing_clients". It is deprecated attribute. Please drop it'%specification.resource_name)

    required_capabilities = 'required_capabilities'
    if required_capabilities in rapp_data:
        data[required_capabilities] = [c for c in rapp_data[required_capabilities]]

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
        d = {k: [] for k in keys}
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
    public_parameters_file_path = _find_resource(public_parameters_resource, rospack)

    # TODO: Do it!

    return d


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
    standard_args = ['gateway_name', 'application_namespace', 'rocon_uri', 'capability_server_nodelet_manager_name']

    try:
        loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False)
        unused_config = load_config_default([roslaunch_file], None, loader=loader, verbose=False, assign_machines=False)
        available_args = [str(x) for x in loader.root_context.resolve_dict['arg']]
        return [x for x in available_args if x in standard_args]
    except (RLException, rospkg.common.ResourceNotFound) as e:
        # The ResourceNotFound lets us catch errors when the launcher has invalid
        # references to resources
        console.logerr("Rapp Indexer : failed to parse top-level args from rapp " +
                     "launch file [" + str(e) + "]")
        return []
