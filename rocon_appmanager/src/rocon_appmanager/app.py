#!/usr/bin/env python             
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot, Daniel Stonier, Jihoon Lee
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#    * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided
#        with the distribution.
#    * Neither the name of Yujin Robot nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import errno
import yaml
import utils
import rospy
import roslaunch.parent
import roslaunch.pmon

from .exceptions import AppException
from roslib.packages import InvalidROSPkgException




"""
    App - Jihoon Lee(jihoonl@yujinrobot.com)

    Got many inspiration and imported from willow_app_manager implementation

    Feature :

    Todo:
"""

class Interface(object):
    data = {}
    
    def __init__(self,data):
        with open(data,'r') as f:
            y = yaml.load(f.read())
            y = y or {}
            try:
                self.data['subscribed_topics'] = y.get('subscribed_topics',{})
                self.data['published_topics'] = y.get('published_topics',{})
            except KeyError:
                raise AppException("Invalid interface, missing keys")

    def __repr__(self):
        string = "\n"
        for slot in self.data:
            string += slot + " : " + str(self.data[slot]) + "\n"

        return string


class App(object):
    path = None 
    data = {}

    def __init__(self,app_name,path):
        self.path = path
        self.load(path,app_name)

    def __repr__(self):
        string = ""
        for d in self.data:
            string += d + " : " + str(self.data[d]) + "\n"
        return string

    def load(self,path,app_name):
        print "Loading App from " + str(path)

        with open(path,'r') as f:
            data = {}
            app_data = yaml.load(f.read()) 

            for reqd in ['launch','interface','platform']:
                if not reqd in app_data:
                    raise AppException("Invalid appfile format [" + path + "], missing required key ["+reqd+"]")

            data['name'] = app_name
            data['display'] =app_data.get('display',app_name)
            data['description'] = app_data.get('description','')
            data['platform'] = app_data['platform']
            data['launch'] = self.loadFromFile(app_data['launch'],'launch',app_name)
            data['interface'] = Interface(self.loadFromFile(app_data['interface'],'interface',app_name))
            if 'icon' not in app_data:
                data['icon'] = None
            else:
                data['icon'] = self.loadFromFile(app_data['icon'],'icon',app_name)
            data['status'] = 'Ready'

        self.data = data
            
    def loadFromFile(self,path,log,app_name="Unknown"):
        try:
            data = utils.findResource(path)
            if not os.path.exists(data):
                raise AppException("Invalid appfile [%s]: %s file does not exist."%(app_name,log))
            return data
        except ValueError as e:
            raise AppException("Invalid appfile [%s]: bad %s entry: %s"%(app_name,log,e))
            """
        except NotFoundException: 
            raise AppException("App file [%s] feres to %s that is not installed"%(app_name,log))
            """
        except InvalidROSPkgException as e:
            raise AppException("App file [%s] feres to %s that is not installed: %s"%(app_name,log,str(e)))
            
    def start(self):
        data = self.data
        rospy.loginfo("Launching: %s"%(data['name']))

        try:
            self._launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"),[data['launch']],is_core=False,process_listeners=())
            self._launch._load_config()
    
            self._launch.start()
            data['status'] = 'Running'
        except Exception as e:
            print str(e)
            rospy.loginfo("Error While launching "+ data['launch'])
            data['status'] = "Error While launching "+ data['launch']
            return False, "Error while launching " + data['name']

        return True, "Success"

    def stop(self):
        data = self.data

        try:
            if self._launch:
                try:
                    self._launch.shutdown()
                finally:
                    self._launch = None
                    data['status'] = 'Ready'
                rospy.loginfo("Stopped App : " + data['name'])
        except Exception as e:
            print str(e)
            rospy.loginfo("Error while stopping " + data['name'])
            data['status'] = 'Error'
            return False, "Error while stopping " + data['name']

        return True , "Success"
            

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
