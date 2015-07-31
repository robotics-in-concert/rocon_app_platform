#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
##############################################################################

import rospy
import roslaunch.parent
import roslib.names
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_python_comms
import copy
from .exceptions import MissingCapabilitiesException


##############################################################################
# Constants
##############################################################################

plurality_converter = {}
plurality_converter["subscribers"] = rocon_python_comms.SUBSCRIBER
plurality_converter["publishers"] = rocon_python_comms.PUBLISHER
plurality_converter["services"] = rocon_python_comms.SERVICE
plurality_converter["action_clients"] = rocon_python_comms.ACTION_CLIENT
plurality_converter["action_servers"] = rocon_python_comms.ACTION_SERVER

##############################################################################
# Utilities
##############################################################################


def dict_to_KeyValue(d):
    '''
      Converts a dictionary to key value ros msg type.

      :param d: dictionary
      :type d: dict

      :returns: KeyValue ros message
      :rtype: [rocon_std_msgs.KeyValue]
    '''
    l = []
    for k, v in d.iteritems():
        l.append(rocon_std_msgs.KeyValue(k, str(v)))
    return l


def _prepare_launch_text(launch_file, launch_args, public_parameters, application_namespace,
                         gateway_name, rocon_uri_string, simulation, capability_server_nodelet_manager_name=None):
    '''
      Prepare the launch file text. This essentially wraps the rapp launcher
      with the following roslaunch elements:

      - group namespace
      - roslaunch args

      The roslaunch args are a list of string keys used to select which
      automatically generated roslaunch args (e.g. namespace, name,
      rocon_uri properties) should get passed to the rapp. Note that we
      don't pass the whole set, because roslaunch args will throw an error
      if the rapp itself isn't expecting them. The logoc for determing this is
      in get_standard_args.

      :param launch_file: fully resolved launch file path
      :type launch_file: str
      :param launch_args: strings identifying the keys of the standard roslaunch args
             to send (not the args themselves)
      :type launch_args: [str]
      :param application_namespace: unique name granted indirectly via the
             gateways, we namespace everything under this
      :type application_namespace: str
      :param gateway_name: unique name granted to the gateway
      :type gateway_name: str
      :param rocon_uri_string: used to pass down information about the platform that is running this app to the app itself.
      :type rocon_uri_string: str - a rocon uri string
      :param simulation: true if rapp manager is for simulated robot
      :type simulation: boolen

      The rocon_uri_string variable is a fixed identifier for this app manager's platform - i.e. no special
      characters or wildcards should be contained therein.
    '''
    # Prepare argument mapping
    launch_arg_mapping = {}
    launch_arg_mapping['application_namespace'] = application_namespace
    if gateway_name:
        launch_arg_mapping['gateway_name'] = gateway_name.lower().replace(' ', '_')
    launch_arg_mapping['rocon_uri'] = rocon_uri_string
    launch_arg_mapping['capability_server_nodelet_manager_name'] = capability_server_nodelet_manager_name
    launch_arg_mapping['simulation'] = simulation

    if(application_namespace == ""):
        launch_text = '<launch>\n  <include file="%s">\n' % (launch_file)
    else:
        launch_text = '<launch>\n  <include ns="%s" file="%s">\n' % (application_namespace, launch_file)

    for arg in launch_args:
        launch_text += '    <arg name="%s" value="%s"/>' % (arg, launch_arg_mapping[arg])
    for name, value in public_parameters.items():
        launch_text += '    <arg name="%s" value="%s"/>' % (name, value)

    launch_text += '  </include>\n</launch>\n'
    return launch_text


def resolve_chain_remappings(nodes):
    '''
        Resolve chain remapping rules contained in node remapping arguments
        replace the node remapping argument

        :param nodes: roslaunch nodes
        :type nodes: roslaunch.Nodes[]
    '''
    for n in nodes:
        new_remap_args_dict = {}

        for fr, to in n.remap_args:
            if str(fr) in new_remap_args_dict:
                rospy.logwarn("Rapp Manager : Remapping rule for %s already exists. Ignoring remapping rule from %s to %s", str(fr), str(fr), str(to))
            else:
                if fr == to:
                    rospy.logwarn("Rapp Manager : ignoring uncessary remapping rule (%s, %s)"%(str(fr),str(to)))
                else:
                    # if there is a value which matches with remap_from, the value should be replaced with the new remap_to
                    keys = [k for k, v in new_remap_args_dict.items() if v == str(fr)]
                    for k in keys:
                        new_remap_args_dict[k] = str(to)
                    new_remap_args_dict[str(fr)] = str(to)
        n.remap_args = new_remap_args_dict.items()


def prepare_launcher(data, public_parameters, application_namespace, gateway_name, rocon_uri_string, capability_nodelet_manager_name, force_screen, simulation, temp):
    '''
      prepare roslaunch to start rapp.
    '''
    # Create modified roslaunch file include the application namespace (robot name + 'application')

    launch_text = _prepare_launch_text(data['launch'],
                                       data['launch_args'],
                                       public_parameters,
                                       application_namespace,
                                       gateway_name,
                                       rocon_uri_string,
                                       simulation,
                                       capability_nodelet_manager_name
                                       )
    temp.write(launch_text)
    temp.close()  # unlink it later

    launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"),
                                              [temp.name],
                                              is_core=False,
                                              process_listeners=(),
                                              force_screen=force_screen)
    launch._load_config()

    return launch


def apply_remapping_rules_from_capabilities(launch_spec, data, caps_list):
    '''
      applies remapping rules from capabilities

      :param launch_spec: rapp launch specification
      :type launch_spec: roslaunch.parent.ROSLaunchParent
      :param data: rapp data
      :type data: dict
      :param caps_list: this holds the list of available capabilities, if app needs capabilities
      :type caps_list: CapsList
      :raises MissingCapabilitiesException: One or more capabilities are unavailable
      :raises rospy.ServiceException: Coulnd't get cap remappings from capability server
    '''
    caps_remap_from_list = []  # contains the app's topics/services/actions to be remapped
    caps_remap_to_list = []  # contains the new topic/service/action names
    for cap in data['required_capabilities']:  # get capability-specific remappings
        rospy.loginfo("Rapp Manager : Configuring remappings for capabilty '" + cap['name'] + "'.")
        try:
            caps_list.get_cap_remappings(cap, caps_remap_from_list, caps_remap_to_list)
        except MissingCapabilitiesException as mis_cap_exc:
            raise mis_cap_exc
        except rospy.ServiceException as srv_exc:
            raise srv_exc
    for node in launch_spec.config.nodes:  # apply remappings to all nodes
        for cap_remap in caps_remap_from_list:
            remap = [unicode(cap_remap), unicode(caps_remap_to_list[caps_remap_from_list.index(cap_remap)])]
            rospy.loginfo("App Manager : Will remap '" + remap[0]
                          + "' to '" + remap[1] + "'.")
            node.remap_args.append(remap)


def apply_remapping_rules_from_start_app_request(launch_spec, data, remappings, application_namespace):
    '''
      applies remapping rules which are requested by start_app service

      :param launch_spec: rapp launch specification
      :type launch_spec: roslaunch.parent.ROSLaunchParent
      :param data: rapp data
      :type data: dict
      :param remapping: rules for the app flips.
      :type remapping: list of rocon_std_msgs.msg.Remapping values.
      :param application_namespace: unique name granted indirectly via the gateways, we namespace everything under this
      :type application_namespace: str

      :returns: Remapped public_interface dictionary
      :rtype: { connection_type: Remapping topic list}
    '''
    connections = {}
    published_interfaces = []

    # Prefix with robot name by default (later pass in remap argument)
    remap_from_list = [remapping.remap_from for remapping in remappings]
    remap_to_list = [remapping.remap_to for remapping in remappings]

    for connection_type in ['publishers', 'subscribers', 'services', 'action_clients', 'action_servers']:
        connections[connection_type] = []
        for t in data['public_interface'][connection_type]:
            if not type(t) is dict:
                rospy.logwarn("Rapp Manager : Public interface has deprecated format. Please update %s includes name and type" % t)
                interface_name = t
                interface_type = 'unknown'
            else:
                interface_name = t['name']
                interface_type = t['type']
            # this is ugly
            published_interface = rapp_manager_msgs.PublishedInterface(
                    interface=rapp_manager_msgs.PublicInterface(plurality_converter[connection_type], interface_type, interface_name),
                    name=''
            )
            remapped_name = None
            # Now we push the rapp launcher down into the prefixed
            # namespace, so just use it directly
            indices = [i for i, x in enumerate(remap_from_list) if x == interface_name]
            if indices:
                if roslib.names.is_global(remap_to_list[indices[0]]):
                    remapped_name = remap_to_list[indices[0]]
                else:
                    remapped_name = '/' + application_namespace.strip("/") + "/" + remap_to_list[indices[0]]
                remapped_name = "/" + remapped_name.lstrip("/")  # ensure only one leading slash
                for N in launch_spec.config.nodes:
                    N.remap_args.append((interface_name, remapped_name))
                connections[connection_type].append(remapped_name)
                published_interface.name = remapped_name
            else:
                # don't pass these in as remapping rules - they should map fine for the node as is
                # just by getting pushed down the namespace.
                #     https://github.com/robotics-in-concert/rocon_app_platform/issues/61
                # we still need to pass them back to register for flipping though.
                if roslib.names.is_global(interface_name):
                    flipped_name = interface_name
                else:
                    flipped_name = application_namespace + '/' + interface_name
                flipped_name = "/" + flipped_name.lstrip("/")  # ensure only one leading slash
                connections[connection_type].append(flipped_name)
                published_interface.name = flipped_name
            published_interfaces.append(published_interface)
    return connections, published_interfaces


def apply_requested_public_parameters(default_parameters, requested_parameters):
    '''
    validate the requested public parameters, and apply them

    :param default_parameters: default public parameters written in rapp specification
    :type default_parameters: dict
    :param requested_parameters: given from start_rapp request
    :type requested_parameters: [rocon_std_msgs.KeyValue]

    :returns: A resolved public_parameters
    :rtype: {name: value}
    '''

    public_parameters = copy.deepcopy(default_parameters)

    # validate whether requested parameters are in public parameter list
    valid_params = {param.key: param.value for param in requested_parameters if param.key in default_parameters}
    invalid_params = [param for param in requested_parameters if not param.key in default_parameters]

    if invalid_params:
        rospy.logwarn('Rapp Manager : Skipping invalid public parameters[%s]' % str(invalid_params))

    for key, val in valid_params.items():
        public_parameters[key] = val

    return public_parameters
