#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_app_manager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import yaml
from roslib.packages import InvalidROSPkgException
import rospy
import roslaunch.parent
import traceback
import tempfile
import rocon_utilities
import utils
from .exceptions import AppException, InvalidRappException
import rocon_app_manager_msgs.msg as rapp_manager_msgs

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
    path = None
    data = {}

    def __init__(self, resource_name):
        '''
          @param resource_name : a package/name pair for this rapp
          @type str/str
        '''
        self.filename = ""
        self._connections = {}
        for connection_type in ['publishers', 'subscribers', 'services', 'action_clients', 'action_servers']:
            self._connections[connection_type] = []
        self._load_from_resource_name(resource_name)

    def __repr__(self):
        string = ""
        for d in self.data:
            string += d + " : " + str(self.data[d]) + "\n"
        return string

    def _load_from_resource_name(self, name):
        '''
          Loads from a ros resource name consisting of a package/app pair.

          @param name : unique identifier for the app, e.g. rocon_apps/chirp.
          @type str

          @raise InvalidRappException if the app definition was for some reason invalid.
        '''
        if not name:
            raise InvalidRappException("app name was invalid [%s]" % name)
        self.filename = utils.find_resource(name + '.rapp')
        self._load_from_app_file(self.filename, name)

    def _load_from_app_file(self, path, app_name):
        '''
          Open and read directly from the app definition file (.rapp file).

          @param path : full path to the .rapp file
          @param app_name : unique name for the app (comes from the .rapp filename)
        '''
        rospy.loginfo("App Manager : loading app '%s'" % app_name)  # str(path)
        self.filename = path

        with open(path, 'r') as f:
            data = {}
            app_data = yaml.load(f.read())

            for reqd in ['launch', 'interface', 'platform']:
                if not reqd in app_data:
                    raise AppException("Invalid appfile format [" + path + "], missing required key [" + reqd + "]")

            data['name'] = app_name
            data['display_name'] = app_data.get('display', app_name)
            data['description'] = app_data.get('description', '')
            data['platform'] = app_data['platform']
            data['launch'] = self._find_rapp_resource(app_data['launch'], 'launch', app_name)
            data['interface'] = self._load_interface(self._find_rapp_resource(app_data['interface'], 'interface', app_name))
            data['pairing_clients'] = []
            data['pairing_clients'] = self._load_pairing_clients(app_data, path)
            if 'icon' not in app_data:
                data['icon'] = None
            else:
                data['icon'] = self._find_rapp_resource(app_data['icon'], 'icon', app_name)
            data['status'] = 'Ready'

        self.data = data

    def to_msg(self):
        '''
          Converts this app definition to ros msg format.
        '''
        a = rapp_manager_msgs.App()
        a.name = self.data['name']
        a.display_name = self.data['display_name']
        a.description = self.data['description']
        a.platform = self.data['platform']
        a.status = self.data['status']
        a.icon = utils.icon_to_msg(self.data['icon'])
        for pairing_client in self.data['pairing_clients']:
            a.pairing_clients.append(PairingClient(pairing_client.client_type,
                                       dict_to_KeyValue(pairing_client.manager_data),
                                       dict_to_KeyValue(pairing_client.app_data)))
        return a

    def _find_rapp_resource(self, resource, log, app_name="Unknown"):
        '''
          A simple wrapper around utils.find_resource to locate rapp resources.

          @param resource is a ros resource (package/name)
          @type str
          @param log : string used for log messages when something goes wrong (e.g. 'icon')
          @type str
          @param name : app name, also only used for logging purposes
          @return full path to the resource
          @type str
          @raise AppException: if resource does not exist or something else went wrong.
        '''
        try:
            path_to_resource = utils.find_resource(resource)
            if not os.path.exists(path_to_resource):
                raise AppException("invalid appfile [%s]: %s file does not exist." % (app_name, log))
            return path_to_resource
        except ValueError as e:
            raise AppException("invalid appfile [%s]: bad %s entry: %s" % (app_name, log, e))
            """
        except NotFoundException:
            raise AppException("App file [%s] refers to %s which is not installed"%(app_name,log))
            """
        except InvalidROSPkgException as e:
            raise AppException("App file [%s] refers to %s which is not installed: %s" % (app_name, log, str(e)))

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

    def start(self, application_namespace, remappings=[], force_screen=False):
        '''
          Some important jobs here.

          1) run the rapp launcher under the unique robot name namespace

          This guarantees that flipped entities generate unique node id's that won't collide when communicating
          with each other (refer to https://github.com/robotics-in-concert/rocon_multimaster/issues/136).

          2) Apply remapping rules while ignoring the namespace underneath.

          @param application_namespace ; unique name granted indirectly via the gateways, we namespace everything under this
          @type str
          @param remapping : rules for the app flips.
          @type list of rocon_app_manager_msgs.msg.Remapping values.
          @param force_screen : whether to roslaunch the app with --screen or not
          @type boolean
        '''
        data = self.data
        rospy.loginfo("App Manager : launching: " + (data['name']) + " underneath /" + application_namespace)

        # Starts rapp
        try:
            temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
            launch_text = '''<launch>
  <include ns="%s" file="%s">
    <arg name="application_namespace" value="%s"/>
  </include>
</launch>\n''' % (application_namespace, data['launch'], application_namespace)
            temp.write(launch_text)
            temp.close()  # unlink it later

            # Create roslaunch
            self._launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"),
                                                            [temp.name],
                                                            is_core=False,
                                                            process_listeners=(),
                                                            force_screen=force_screen)
            self._launch._load_config()

            #print data['interface']
            self._connections = {}

            # Prefix with robot name by default (later pass in remap argument)
            remap_from_list = [remapping.remap_from for remapping in remappings]
            remap_to_list = [remapping.remap_to for remapping in remappings]
            for connection_type in ['publishers', 'subscribers', 'services', 'action_clients', 'action_servers']:
                self._connections[connection_type] = []
                for t in data['interface'][connection_type]:
                    remapped_name = None
                    # Now we push the rapp launcher down into the prefixed
                    # namespace, so just use it directly
                    indices = [i for i, x in enumerate(remap_from_list) if x == t]
                    if indices:
                        if rocon_utilities.ros.is_absolute_name(remap_to_list[indices[0]]):
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
                        if rocon_utilities.ros.is_absolute_name(t):
                            flipped_name = t
                        else:
                            flipped_name = '/' + application_namespace + '/' + t
                        self._connections[connection_type].append(flipped_name)
            self._launch.start()

            data['status'] = 'Running'
            return True, "Success", self._connections['subscribers'], self._connections['publishers'], self._connections['services'], self._connections['action_clients'], self._connections['action_servers']

        except Exception as e:
            print str(e)
            traceback.print_stack()
            rospy.loginfo("Error While launching " + data['launch'])
            data['status'] = "Error While launching " + data['launch']
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
                rospy.loginfo("App Manager : stopped app [%s]" % data['name'])
        except Exception as e:
            print str(e)
            rospy.loginfo("Error while stopping " + data['name'])
            data['status'] = 'Error'
            return False, "Error while stopping " + data['name'], self._connections['subscribers'], self._connections['publishers'], self._connections['services'], self._connections['action_clients'], self._connections['action_servers']

        return True, "Success", self._connections['subscribers'], self._connections['publishers'], self._connections['services'], self._connections['action_clients'], self._connections['action_servers']

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
        l.append(rapp_manager_msgs.KeyValue(k, str(v)))
    return l
