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
import utils
import rospy
import roslaunch.parent
import traceback
import time
import thread

from .exceptions import AppException
from roslib.packages import InvalidROSPkgException

"""
    App - Jihoon Lee(jihoonl@yujinrobot.com)

    Got many inspiration and imported from willow_app_manager implementation

    Feature :

    Todo:
"""


class App(object):
    path = None
    data = {}

    def __init__(self, app_name, path):
        self.path = path
        self.load(path, app_name)

    def __repr__(self):
        string = ""
        for d in self.data:
            string += d + " : " + str(self.data[d]) + "\n"
        return string

    def load(self, path, app_name):
        rospy.loginfo("App Manager : loading app '%s'" % app_name)  # str(path)

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
            data['interface'] = self.loadInterface(self.loadFromFile(app_data['interface'], 'interface', app_name))
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

    def loadInterface(self, data):
        d = {}
        keys = ['subscriber', 'publisher', 'service']
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

    def start(self, robot_name):
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

            print data['interface']

            # Remap the topics
            for N in self._launch.config.nodes:
                for t in data['interface']['publisher']:
                    N.remap_args.append((t, prefix + '/' + t))
                for t in data['interface']['subscriber']:
                    N.remap_args.append((t, prefix + '/' + t))
                for t in data['interface']['service']:
                    N.remap_args.append((t, prefix + '/' + t))

            self._launch.start()

            self.subscribers = [prefix + '/' + x for x in data['interface']['subscriber']]
            self.publishers = [prefix + '/' + x for x in data['interface']['publisher']]
            self.services = [prefix + '/' + x for x in data['interface']['service']]

            thread.start_new_thread(self.app_monitor, ())
            data['status'] = 'Running'
            return True, "Success", self.subscribers, self.publishers, self.services

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
            return False, "Error while stopping " + data['name'], self.subscribers, self.publishers, self.services

        return True, "Success", self.subscribers, self.publishers, self.services

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
