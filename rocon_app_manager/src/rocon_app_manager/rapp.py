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
import roslib.names
import rospy
from roslaunch.config import load_config_default
from roslaunch.core import RLException
import roslaunch.parent
import traceback
import tempfile
import rocon_utilities
from .exceptions import AppException, InvalidRappException, MissingCapabilitiesException
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_std_msgs.msg as rocon_std_msgs
import roslaunch.xmlloader

##############################################################################
# Class
##############################################################################


class PairingClient(object):
    '''
      A pairing client runs an app which is one which will work in tandem with
      this robot (rocon) app. The client is usually a smart phone or tablet.
    '''
    __slots__ = ['client_type', 'manager_data', 'app_data']

    def __init__(self, client_type, manager_data, app_data):
        self.client_type = client_type
        self.manager_data = manager_data
        self.app_data = app_data

    def as_dict(self):
        return {'client_type': self.client_type, 'manager_data': self.manager_data, 'app_data': self.app_data}

    def __eq__(self, other):
        if not isinstance(other, PairingClient):
            return False
        return self.client_type == other.client_type and \
               self.manager_data == other.manager_data and \
               self.app_data == other.app_data

    def __repr__(self):
        return yaml.dump(self.as_dict())


class Rapp(object):
    '''
        Got many inspiration and imported from willow_app_manager
        implementation (Jihoon)
    '''
    # I should add a __slots__ definition here to make it easy to read

    # args that can be parsed for inside a rapp launcher, these args
    # get passed down in the temporary launcher that gets constructed when
    # pushing down a namespace.
    standard_args = ['gateway_name', 'application_namespace', 'rocon_uri']

    def __init__(self, package, package_relative_rapp_filename):
        '''
           @param package this rapp is nested in
           @type :py:class:`catkin_pkg.package.Package`
           @param package_relative_rapp_filename : string specified by the package export
           @type os.path
        '''
        self.package_name = package.name
        package_path = os.path.dirname(package.filename)
        self.filename = os.path.join(package_path, package_relative_rapp_filename)
        self._connections = {}
        for connection_type in ['publishers', 'subscribers', 'services', 'action_clients', 'action_servers']:
            self._connections[connection_type] = []
        rapp_name = os.path.splitext(os.path.basename(self.filename))[0]
        rospy.loginfo("App Manager : loading rapp '%s/%s'" % (package.name, rapp_name))
        with open(self.filename, 'r') as f:
            data = {}
            app_data = yaml.load(f.read())
            for reqd in ['launch', 'public_interface', 'compatibility']:
                if not reqd in app_data:
                    raise AppException("Invalid rapp file format [" + self.filename + "], missing required key [" + reqd + "]")
            data['name'] = package.name + "/" + rapp_name
            data['display_name'] = app_data.get('display', rapp_name)
            data['description'] = app_data.get('description', '')
            data['compatibility'] = app_data['compatibility']
            data['launch'] = self._find_rapp_resource(rapp_name, package_path, app_data['launch'])
            data['launch_args'] = get_standard_args(data['launch'])
            data['public_interface'] = self._load_interface(self._find_rapp_resource(rapp_name, package_path, app_data['public_interface']))
            data['pairing_clients'] = self._load_pairing_clients(app_data, self.filename)
            if 'icon' not in app_data:
                data['icon'] = None
            else:
                data['icon'] = self._find_rapp_resource(rapp_name, package_path, app_data['icon'])
            data['status'] = 'Ready'
            required_capabilities = 'required_capabilities'
            if required_capabilities in app_data:
                data[required_capabilities] = []
                for cap in app_data[required_capabilities]:
                    data[required_capabilities].append(cap)
        self.data = data

    def _find_rapp_resource(self, rapp_name, package_path, resource):
        '''
          Find a rapp resource (.launch, .interface, icon) relative to the
          specified package path.

          @param rapp_name : name of the rapp, only used for log messages.
          @type str
          @param package_path : the root of the package to begin the search
          @type os.path
          @param resource : a typical resource identifier to look for
          @type pkg_name/file pair in str format.
        '''
        unused_package, name = roslib.names.package_resource_name(resource)
        for root, dirs, files in os.walk(package_path):
            if name in files:
                return os.path.join(root, name)
            to_prune = [x for x in dirs if x.startswith('.')]
            for x in to_prune:
                dirs.remove(x)
        raise AppException("invalid rapp - %s does not exist [%s]" % (name, rapp_name))

    def __repr__(self):
        string = ""
        for d in self.data:
            string += d + " : " + str(self.data[d]) + "\n"
        return string

    def to_msg(self):
        '''
          Converts this app definition to ros msg format.
        '''
        a = rapp_manager_msgs.App()
        a.name = self.data['name']
        a.display_name = self.data['display_name']
        a.description = self.data['description']
        a.compatibility = self.data['compatibility']
        a.status = self.data['status']
        a.icon = rocon_utilities.icon_to_msg(self.data['icon'])
        for pairing_client in self.data['pairing_clients']:
            a.pairing_clients.append(PairingClient(pairing_client.client_type,
                                     dict_to_KeyValue(pairing_client.manager_data),
                                     dict_to_KeyValue(pairing_client.app_data)))
        key = 'required_capabilities'
        if key in self.data:
            for cap in self.data[key]:
                a.required_capabilities.append(cap['name'])
        return a

    def _load_interface(self, data):
        d = {}
        keys = ['subscribers', 'publishers', 'services', 'action_clients', 'action_servers']
        with open(data, 'r') as f:
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
                raise AppException("Invalid interface, missing keys")

        return d

    def _load_pairing_clients(self, app_data, appfile="UNKNOWN"):
        '''
          Load pairing client information from the .rapp file.

          @raise InvalidRappException if the .rapp pairing clients definition was invalid.
        '''
        clients_data = app_data.get('pairing_clients', [])
        clients = []
        for c in clients_data:
            for reqd in ['type', 'manager']:
                if not reqd in c:
                    raise InvalidRappException("malformed .rapp [%s], missing required key [%s]" % (appfile, reqd))
            client_type = c['type']
            manager_data = c['manager']
            if not type(manager_data) == dict:
                raise InvalidRappException("malformed .rapp [%s]: manager data must be a map" % (appfile))

            app_data = c.get('app', {})
            if not type(app_data) == dict:
                raise InvalidRappException("malformed appfile [%s]: app data must be a map" % (appfile))

            clients.append(PairingClient(client_type, manager_data, app_data))
        return clients

    def start(self, application_namespace, gateway_name, rocon_uri_string, remappings=[], force_screen=False,
              caps_list=None):
        '''
          Some important jobs here.

          1) run the rapp launcher under the unique robot name namespace

          This guarantees that flipped entities generate unique node id's that won't collide when communicating
          with each other (refer to https://github.com/robotics-in-concert/rocon_multimaster/issues/136).

          2) Apply remapping rules while ignoring the namespace underneath.

          @param application_namespace ; unique name granted indirectly via the gateways, we namespace everything under this
          @type str
          @param gateway_name ; unique name granted to the gateway
          @type str
          @param rocon_uri_string : uri of the app manager's platform (used as a check for compatibility)
          @type str : a rocon uri string
          @param remapping : rules for the app flips.
          @type list of rocon_std_msgs.msg.Remapping values.
          @param force_screen : whether to roslaunch the app with --screen or not
          @type boolean
          @param caps_list : this holds the list of available capabilities, if app needs capabilities
          @type CapsList
        '''
        data = self.data
        rospy.loginfo("App Manager : launching '" + (data['name']) + "' underneath /" + application_namespace)

        try:
            # Create modified roslaunch file include the application namespace (robot name + 'application')
            temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
            launch_text = _prepare_launch_text(
                                    data['launch'],
                                    data['launch_args'],
                                    application_namespace,
                                    gateway_name,
                                    rocon_uri_string
                                    )
            temp.write(launch_text)
            temp.close()  # unlink it later

            # initialise roslaunch
            self._launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"),
                                                            [temp.name],
                                                            is_core=False,
                                                            process_listeners=(),
                                                            force_screen=force_screen)
            self._launch._load_config()  # generate configuration

            if 'required_capabilities' in self.data:  # apply capability-specific remappings needed
                caps_remap_from_list = []  # contains the app's topics/services/actions to be remapped
                caps_remap_to_list = []  # contains the new topic/service/action names
                for cap in self.data['required_capabilities']:  # get capability-specific remappings
                    rospy.loginfo("App Manager : Configuring remappings for capabilty '" + cap['name'] + "'.")
                    if caps_list:
                        try:
                            caps_list.get_cap_remappings(cap, caps_remap_from_list, caps_remap_to_list)
                        except MissingCapabilitiesException as e:
                            rospy.logerr("App Manager : Couldn't get cap remappings. Error: " + str(e))
                    else:  # should not happen, since app would have been pruned
                        raise Exception("Capabilities required, but capability list not provided.")
                for cap_remap in caps_remap_from_list:  # apply cap remappings
                    remap_applied = False
                    for node in self._launch.config.nodes:  # look for cap remap topic is remap topics
                        for node_remap in node.remap_args:  # topic from - topic to pairs
                            if cap_remap in node_remap[0]:
                                node_remap[1] = unicode(caps_remap_to_list[caps_remap_from_list.index(cap_remap)])
                                rospy.loginfo("App Manager : Will remap '" + node_remap[0]
                                              + "' to '" + node_remap[1] + "'.")
                                remap_applied = True
                    if not remap_applied:  # can't determine to which the remapping should be applied to
                        rospy.logwarn("App Manager : Could not determine remapping for capability topic '"
                                      + cap_remap + "'.")
                        rospy.logwarn("App Manager : App might not function correctly."
                                   + " Add it to the remapped topics, if needed.")

            self._connections = {}

            # Prefix with robot name by default (later pass in remap argument)
            remap_from_list = [remapping.remap_from for remapping in remappings]
            remap_to_list = [remapping.remap_to for remapping in remappings]

            for connection_type in ['publishers', 'subscribers', 'services', 'action_clients', 'action_servers']:
                self._connections[connection_type] = []
                for t in data['public_interface'][connection_type]:
                    remapped_name = None
                    # Now we push the rapp launcher down into the prefixed
                    # namespace, so just use it directly
                    indices = [i for i, x in enumerate(remap_from_list) if x == t]
                    if indices:
                        if roslib.names.is_global(remap_to_list[indices[0]]):
                            remapped_name = remap_to_list[indices[0]]
                        else:
                            remapped_name = '/' + application_namespace + "/" + remap_to_list[indices[0]]
                        for N in self._launch.config.nodes:
                            N.remap_args.append((t, remapped_name))
                        self._connections[connection_type].append(remapped_name)
                    else:
                        # don't pass these in as remapping rules - they should map fine for the node as is
                        # just by getting pushed down the namespace.
                        #     https://github.com/robotics-in-concert/rocon_app_platform/issues/61
                        # we still need to pass them back to register for flipping though.
                        if roslib.names.is_global(t):
                            flipped_name = t
                        else:
                            flipped_name = '/' + application_namespace + '/' + t
                        self._connections[connection_type].append(flipped_name)
            _resolve_chain_remappings(self._launch.config.nodes)
            self._launch.start()

            data['status'] = 'Running'
            return True, "Success", self._connections['subscribers'], self._connections['publishers'], \
                self._connections['services'], self._connections['action_clients'], self._connections['action_servers']

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

         @return True if the rapp is executing or False otherwise.
         @type Bool
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


def dict_to_KeyValue(d):
    '''
      Converts a dictionary to key value ros msg type.
    '''
    l = []
    for k, v in d.iteritems():
        l.append(rocon_std_msgs.KeyValue(k, str(v)))
    return l


def get_standard_args(roslaunch_file):
    '''
      Given the Rapp launch file, this function parses the top-level args
      in the file. Returns the complete list of top-level arguments that
      match standard args so that they can be passed to the launch file

      @param roslaunch_file : rapp launch file we are parsing for arguments
      @type str
      @return list of top-level arguments that match standard arguments. Empty
              list on parse failure
      @rtype [str]
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


def _prepare_launch_text(launch_file, launch_args, application_namespace,
                        gateway_name, rocon_uri_string):
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

      @param launch_file: fully resolved launch file path
      @type str
      @param launch_args: strings identifying the keys of the standard roslaunch args
             to send (not the args themselves)
      @type str[]
      @param application_namespace ; unique name granted indirectly via the
             gateways, we namespace everything under this
      @type str
      @param gateway_name : unique name granted to the gateway
      @type str
      @param rocon_uri_string : used to pass down information about the platform that is running this app to the app itself.
      @type str : a rocon uri string

      The rocon_uri_string variable is a fixed identifier for this app manager's platform - i.e. no special
      characters or wildcards should be contained therein.
    '''

    # Prepare argument mapping
    launch_arg_mapping = {}
    launch_arg_mapping['application_namespace'] = application_namespace
    launch_arg_mapping['gateway_name'] = gateway_name
    launch_arg_mapping['rocon_uri'] = rocon_uri_string

    launch_text = '<launch>\n  <include ns="%s" file="%s">\n' % (application_namespace, launch_file)
    for arg in launch_args:
        launch_text += '    <arg name="%s" value="%s"/>' % (arg, launch_arg_mapping[arg])
    launch_text += '  </include>\n</launch>\n'
    return launch_text


def _resolve_chain_remappings(nodes):
    """
        Resolve chain remapping rules contained in node remapping arguments
        replace the node remapping argument

        @param nodes: roslaunch nodes
        @type: roslaunch.Nodes[]
    """
    for n in nodes:
        new_remap_args_dict = {}

        for fr, to in n.remap_args:
            if str(fr) in new_remap_args_dict:
                rospy.logwarn("App Manager : Remapping rule for %s already exists. Ignoring remapping rule from %s to %s", str(fr), str(fr), str(to))
            else:
                # if there is a value which matches with remap_from, the value should be replaced with the new remap_to
                keys = [k for k, v in new_remap_args_dict.items() if v == str(fr)]
                for k in keys:
                    new_remap_args_dict[k] = str(to)

                new_remap_args_dict[str(fr)] = str(to)
        n.remap_args = new_remap_args_dict.items()
