#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_appmanager/LICENSE
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
import time
import thread

import utils
from .exceptions import AppException, InvalidAppException

##############################################################################
# Class
##############################################################################


class App(object):
    '''
        Got many inspiration and imported from willow_app_manager
        implementation (Jihoon)
    '''
    path = None
    data = {}

    def __init__(self):
        self.filename = ""
        self._connections = {}
        for connection_type in ['publishers', 'subscribers', 'services']:
            self._connections[connection_type] = []

    def __repr__(self):
        string = ""
        for d in self.data:
            string += d + " : " + str(self.data[d]) + "\n"
        return string

    def load_from_resource_name(self, name):
        '''
          Loads from a ros resource name consisting of a package/app pair.

          @param name : unique identifier for the app, e.g. rocon_apps/chirp.
          @type str

          @raise InvalidAppException if the app definition was for some reason invalid.
        '''
        if not name:
            raise InvalidAppException("app name was invalid [%s]" % name)
        self.filename = utils.find_resource(name + '.app')
        return self.load_from_app_file(self.filename, name)

    def load_from_app_file(self, path, app_name):
        '''
          Open and read directly from the app definition file (.app file).

          @param path : full path to the .app file
          @param app_name : unique name for the app (comes from the .app filename)
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
            data['display'] = app_data.get('display', app_name)
            data['description'] = app_data.get('description', '')
            data['platform'] = app_data['platform']
            data['launch'] = self.loadFromFile(app_data['launch'], 'launch', app_name)
            data['interface'] = self._load_interface(self.loadFromFile(app_data['interface'], 'interface', app_name))
            if 'icon' not in app_data:
                data['icon'] = None
            else:
                data['icon'] = self.loadFromFile(app_data['icon'], 'icon', app_name)
            data['status'] = 'Ready'

        self.data = data

    def loadFromFile(self, path, log, app_name="Unknown"):
        try:
            data = utils.findResource(path)
            if not os.path.exists(data):
                raise AppException("Invalid appfile [%s]: %s file does not exist." % (app_name, log))
            return data
        except ValueError as e:
            raise AppException("Invalid appfile [%s]: bad %s entry: %s" % (app_name, log, e))
            """
        except NotFoundException:
            raise AppException("App file [%s] feres to %s that is not installed"%(app_name,log))
            """
        except InvalidROSPkgException as e:
            raise AppException("App file [%s] feres to %s that is not installed: %s" % (app_name, log, str(e)))

    def _load_interface(self, data):
        d = {}
        keys = ['subscribers', 'publishers', 'services']
        with open(data, 'r') as f:
            y = yaml.load(f.read())
            y = y or {}
            try:
                for k in keys:
                    raw_data = y.get(k, [])

                    # remote / in front of topics
                    new_data = []
                    for r in raw_data:
                        if r[0] == '/':
                            r = r[1:len(r)]
                        new_data.append(r)
                    d[k] = new_data

            except KeyError:
                raise AppException("Invalid interface, missing keys")

        return d

    def start(self, robot_name, remappings=[]):
        data = self.data
        rospy.loginfo("Launching: %s" % (data['name']))

        # Starts app
        try:
            prefix = robot_name

            # Create roslaunch
            self._launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"),
                                                            [data['launch']],
                                                            is_core=False,
                                                            process_listeners=())
            self._launch._load_config()

            #print data['interface']
            self._connections = {}

            # Prefix with robot name by default (later pass in remap argument)
            remap_from_list = [remapping.remap_from for remapping in remappings]
            remap_to_list = [remapping.remap_to for remapping in remappings]
            for connection_type in ['publishers', 'subscribers', 'services']:
                self._connections[connection_type] = []
                for t in data['interface'][connection_type]:
                    remapped_name = prefix + '/' + t
                    indices = [i for i, x in enumerate(remap_from_list) if x == t]
                    if indices:
                        remapped_name = remap_to_list[indices[0]]
                    self._connections[connection_type].append(remapped_name)
                    for N in self._launch.config.nodes:
                        N.remap_args.append((t, remapped_name))
            self._launch.start()

            thread.start_new_thread(self.app_monitor, ())
            data['status'] = 'Running'
            return True, "Success", self._connections['subscribers'], self._connections['publishers'], self._connections['services']

        except Exception as e:
            print str(e)
            traceback.print_stack()
            rospy.loginfo("Error While launching " + data['launch'])
            data['status'] = "Error While launching " + data['launch']
            return False, "Error while launching " + data['name'], [], [], []

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
            return False, "Error while stopping " + data['name'], self._connections['subscribers'], self._connections['publishers'], self._connections['services']

        return True, "Success", self._connections['subscribers'], self._connections['publishers'], self._connections['services']

    def app_monitor(self):
        while self._launch:
            time.sleep(0.1)
            launch = self._launch
            if launch:
                pm = launch.pm
                if pm:
                    if pm.done:
                        time.sleep(1.0)
                        self.stop()
                        break
