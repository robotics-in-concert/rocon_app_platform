#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################
from .exceptions import InvalidRappException, RappResourceNotExistException, RappMalformedException, XmlParseException
import copy
import os
import yaml
import rospkg
from xml.dom.minidom import parse
from xml.dom import Node as DomNode
import rocon_python_utils
from rocon_console import console

def load_rapp_yaml_from_file(filename):
    '''
      Load rapp specs yaml from the given file

      :param filename: absolute path to the rapp definition
      :type filename: str

      :returns: raw yaml info, dict of loaded rapp
      :rtype: dict, dict

      :raises: InvalidRappException: Rapp includes invalid filed
    '''
    RAPP_ATTRIBUTES = ['display', 'description', 'icon', 'public_interface', 'public_parameters', 'compatibility', 'launch', 'parent_name', 'pairing_clients', 'required_capabilities']
    base_path = os.path.dirname(filename)

    with open(filename, 'r') as f:
        yaml_data = yaml.load(f.read())
        app_data = copy.deepcopy(yaml_data)

        for d in app_data:
            if d not in RAPP_ATTRIBUTES:
                raise InvalidRappException('Invalid Field : [' + str(d) + '] Valid Fields : [' + str(RAPP_ATTRIBUTES) + ']')

        if 'launch' in app_data:
            app_data['launch'] = _find_resource(base_path, app_data['launch'])
            yaml_data['launch'] = app_data['launch']
        if 'public_interface' in app_data:
            yaml_data['public_interface'], app_data['public_interface']  = _load_public_interface(base_path, app_data['public_interface'])
        if 'public_parameters' in app_data:
            yaml_data['public_parameters'], app_data['public_parameters']  = _load_public_parameters(base_path, app_data['public_parameters'])
        if 'icon' in app_data:
            app_data['icon'] = _find_resource(base_path, app_data['icon'])
            yaml_data['icon'] = app_data['icon']
    return yaml_data, app_data


def load_rapp_specs_from_file(specification):
    '''
      Specification consists of resource which is file pointer. This function loads those files in memeory

      :param specification: Rapp Specification
      :type Specification: rocon_app_utilities.Rapp

      :returns Fully loaded rapp data dictionary
      :rtype: dict
    '''
    base_path = os.path.dirname(specification.filename)
    rapp_data = specification.raw_data

    data = {}
    data['name'] = specification.resource_name
    data['ancestor_name'] = specification.ancestor_name
    data['display_name']      = rapp_data.get('display', data['name'])
    data['description']       = rapp_data.get('description', '')
    data['compatibility']     = rapp_data['compatibility']
    data['launch']            = rapp_data['launch']
    data['launch_args']       = _get_standard_args(data['launch'])
    data['public_interface']  = rapp_data.get('public_interface', _default_public_interface())
    data['public_parameters'] = rapp_data.get('public_parameters', {})
    data['icon']              = rapp_data.get('icon', None)

    if 'pairing_clients' in rapp_data:
        console.logwarn('Rapp Indexer : [%s] includes "pairing_clients". It is deprecated attribute. Please drop it'%specification.resource_name)

    required_capabilities = 'required_capabilities'
    if required_capabilities in rapp_data:
        data[required_capabilities] = [c for c in rapp_data[required_capabilities]]

    return data


def _find_resource(base_path, resource):
    '''
      Find a rapp resource (.launch, .interface, icon) relative to the
      specified package path.

      :param base_path: relative path to resource 
      :type base_path: str
      :param resource: a typical resource identifier to look for
      :type resource: pkg_name/file pair in str format.

      :raises: :exc:`.exceptions.RappResourceNotExistException` if the resource is not found
    '''
    path = os.path.join(base_path, resource)
    if os.path.exists(path):
        return os.path.normpath(path)
    else:
        try:
            found = rocon_python_utils.ros.find_resource_from_string(resource)
        except rospkg.ResourceNotFound:
            raise RappResourceNotExistException("invalid rapp - %s does not exist" % (resource))
        raise RappResourceNotExistException("invalid rapp - %s is 'tuple based rapp resource'. It is deprecated attribute. Please fix it as relative path to .rapp file" % (resource))


def _default_public_interface():
    '''
    returns dictionary of public_interface keys
    '''
    d = {}
    keys = ['subscribers', 'publishers', 'services', 'action_clients', 'action_servers']

    d = {k: [] for k in keys}
    return d


def _load_public_interface(base_path, public_interface_resource):
    '''
      loading public interfaces from file. If the given filepath

      :param base_path: relative path to resource 
      :type base_path: str
      :params public_interface_resource: relative path of public interface file
      :type: str

      :returns: resource path, dict of public interface
      :rtype: os.path, {keys : []}

      :raises RappMalformedException: public interface contains mssing key
    '''
    d = {}
    keys = ['subscribers', 'publishers', 'services', 'action_clients', 'action_servers']

    # If public interface is not present. return empty list for each connection types
    if not public_interface_resource:
        d = {k: [] for k in keys}
        return None, d

    public_interface_file_path = _find_resource(base_path, public_interface_resource)
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
    return public_interface_file_path, d


def _load_public_parameters(base_path, public_parameters_resource):
    '''
      :param base_path: relative path to resource 
      :type base_path: str
      :params public_parameters_resource: relative path of public parameters file
      :type: str

      :returns: resource path, dict of public parameter
      :rtype: os.path, {keys : []}
    '''

    # If public interface is not present. return empty list for each connection types
    if not public_parameters_resource:
        return None, {}

    public_parameters_file_path = _find_resource(base_path, public_parameters_resource)
    with open(public_parameters_file_path, 'r') as f:
        y = yaml.load(f.read())
        y = y or {}
        
    return public_parameters_file_path, y


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

      :raises RappMalformedException: if launch file format is invalid
    '''
    standard_args = ['gateway_name', 'application_namespace', 'rocon_uri', 'capability_server_nodelet_manager_name', 'simulation']

    try:
        available_args = _get_available_args(roslaunch_file)
        return [x for x in available_args if x in standard_args]
    except (XmlParseException) as e:
        # The ResourceNotFound lets us catch errors when the launcher has invalid
        # references to resources
        reason = "failed to parse top-level args from rapp " + "launch file [" + str(e) + "]"
        raise RappMalformedException(str(reason))


def _get_available_args(filename):
    '''
      Load XML from file to extract top-level args and returns available_args

      :param filename: rapp launch file we are parsing for arguements
      :type filename: str

      :returns: list of available args 
      :rtype: [str]
    '''
    xml = _parse_launch(filename)
    available_args = [node.attributes['name'].value.strip() for node in xml.childNodes if node.nodeType == DomNode.ELEMENT_NODE and node.tagName == 'arg']
    return available_args 

def _parse_launch(filename):
    '''
      Parse launch from file

      :param filename: rapp launch file we are parsing for arguements
      :type filename: str
      :returns: launch XML
      :ortype: XML

      :raises XmlParseException: if xml is invalid format
    '''
    try:
        root = parse(filename).getElementsByTagName('launch')
    except Exception as e:
        raise XmlParseException('Invalid roslaunch XML syntax: %s'%e)
    if len(root) != 1:
        raise XmlParseException('Invalid roslaunch XML syntax: no root <launch> tag')
    return root[0]
