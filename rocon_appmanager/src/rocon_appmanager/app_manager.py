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

import rospy
import os
import sys
import roslaunch.pmon
from .app import App
from .logger import Logger
from appmanager_comms.srv import *
from appmanager_comms.msg import *
from std_msgs.msg import String
from gateway_comms.srv import *
from gateway_comms.msg import ConnectionType,Rule,RemoteRule

#from concert_client.concert_client import ConcertClient
"""
    AppManager - Jihoon Lee(jihoonl@yujinrobot.com)

    Feature:
        Configuration
            - app_list_directory by yaml(Done)

        App Management
            - Load installed apps from app_list directory. 
            - launches app

    Todo:
        Configuration
            - app_store url            by yaml

        Concert Master Relation(in concert_client.py)
            - When a new concert master comes in network, it flips out publisher that informs platform-info, and service to listen an invitation.
            - When it receives a invitation from concert master, it validates the inviter with it's white/black list, then closes channels to other concert masters.
            - Have install/uninstall/start/stop an app services
    
        App Management
            - have publisher that send out installed app list, available app list, and some more info. maybe latched=true one.
            - 'apt-get' app from app store
            - cache-ing the app lists

"""

#class AppManager(ConcertClient):
class AppManager(object):

    param = {}
    apps = {}
    app_list = None
    DEFAULT_APP_LIST_DIRECTORY = '/opt/ros/fuerte/stacks/'

    listapp_srv_name = '~list_apps'
    start_app_srv_name = '~start_app'
    stop_app_srv_name = '~stop_app'
    log_pub_name = '~log'

    services = {}
    pubs = {}
    gateway_srvs = {}

    def __init__(self):

        # load configuration from rosparam
        rospy.loginfo("Parsing Parameters")
        self.parseParams()

#        if self.param['is_alone'] == False:
#            super(AppManager,self).__init__(self.param['white_list'], self.param['black_list'],self.param['platform_info'])

        # It sets up an app directory and load installed app list from directory
        rospy.loginfo("Loading app lists")
        self.getInstalledApplist()
        rospy.loginfo("Done")

        rospy.loginfo("Advertising Services");
        self.services = {}
        self.services['get_applist'] = rospy.Service(self.listapp_srv_name,GetAppList,self.processGetAppList)
        self.services['start_app'] = rospy.Service(self.start_app_srv_name,StartApp,self.processStartApp)
        self.services['stop_app'] = rospy.Service(self.stop_app_srv_name,StopApp,self.processStopApp)

        self.pubs = {}
        self.pubs['log'] = rospy.Publisher(self.log_pub_name,String)

        roslaunch.pmon._init_signal_handlers()

        # Logging mechanism. hooks std_out and publish as a topic
        self.logger = Logger(sys.stdout)
        sys.stdout = self.logger
        self.logger.addCallback(self.process_stdmsg)

        # flips or advertise list_apps, start_app, stop_app to outdoor
        self.setGatewaySrvs()

        rospy.loginfo("Advertising appmanager apis")

    def setGatewaySrvs(self):
        self.gateway_srv = {}
        self.gateway_srv['flip'] =  rospy.ServiceProxy('/gateway/flip',Remote)
#        self.gateway_srv['flip_all'] = rospy.ServiceProxy('/gateway/flip_all',RemoteAll)
        self.gateway_srv['advertise'] = rospy.ServiceProxy('/gateway/advertise',Advertise)
#        self.gateway_srv['advertise_all'] = rospy.ServiceProxy('/gateway/advertise',AdvertiseAll)
        self.gateway_srv['pull'] = rospy.ServiceProxy('/gateway/pull',Remote)

    def makePublic(self,public_flag):
        # advertise list_apps,start_app, and stop_app
        name = rospy.get_name()

        services = [name+'/list_apps',name+'/start_app',name+'/stop_app',name+'/log']
        publishers = [name+'/log']

        self.flips(services,ConnectionType.SERVICE,public_flag)
        self.flips(publishers,ConnectionType.PUBLISHER,public_flag)


    def createRemoteRule(self,gateway,rule):
        r = RemoteRule()
        r.gateway = gateway
        r.rule = rule

        return r

    def createRule(self,name,type):
        r = Rule()
        r.name = name 
        r.type = type
        r.node = ''
        return r
                
    def parseParams(self):
        param = {}
        param['robot_name'] = rospy.get_param('~robot_name')
        param['app_from_source_directory'] = rospy.get_param('~app_from_source_directory',self.DEFAULT_APP_LIST_DIRECTORY)
        param['app_store_url'] = rospy.get_param('~app_store_url','')
        param['platform_info'] = rospy.get_param('~platform_info','')
        param['white_list'] = rospy.get_param('~whitelist','')
        param['black_list'] = rospy.get_param('~black_list','')
        param['is_alone'] = rospy.get_param('~is_alone',False)
        param['remote_name'] = rospy.get_param('~remote_name','pirate_gateway1')

        self.param = param


    # it sets up an app directory and load installed app from given directory
    def getInstalledApplist(self):
        apps = {}
        apps_str = {}
        
        # Getting apps from source
        directory = self.param['app_from_source_directory']
        apps_str['from_source'] = self.load(directory,'.app')
        apps['from_source'] = {}
        for app_str in apps_str['from_source']:
            try:
                apps['from_source'][app_str[0]] = App(app_str[0],app_str[1])
            except Exception as e:
                print str(e)
#                del apps['from_source'][app_str[0]]



        # Getting apps in store
#print str(apps['from_source'].keys())

        self.apps = apps
        self.apps_str = apps_str


    # It searchs *.app in directories
    def load(self,directory,typ):
        applist = []
        for dpath,_ ,files in os.walk(directory):
            apps = [f for f in files if f.endswith(typ)]
            apps_with_path = [dpath + '/' + a for a in apps]
            apps_name = [a[0:len(a)-len(typ)] for a in apps]

            applist += list(zip(apps_name,apps_with_path))

        return applist                

    def processGetAppList(self,req):
        apps_description = GetAppListResponse()

        for app_name in self.apps['from_source']:
            app = self.apps['from_source'][app_name]
            a = AppDescription()
            a.name = app.data['name']
            a.display = app.data['display']
            a.description = app.data['description']
            a.display = app.data['platform']
            a.status = app.data['status']
            apps_description.apps.append(a)

        return apps_description

    def processStartApp(self,req):
        
        rospy.loginfo("Starting App : " + req.name)
        resp = StartAppResponse()
        resp.started, resp.message, subscribers, publishers,services = self.apps['from_source'][req.name].start(self.param['robot_name'])

#        self.pullinTopics(pullin_topics,True)
        self.flips(subscribers,ConnectionType.SUBSCRIBER,True)
        self.flips(publishers,ConnectionType.PUBLISHER,True)
        self.flips(services,ConnectionType.SERVICE,True)

        return resp


    def processStopApp(self,req):
        rospy.loginfo("Stopping App : " + req.name)
        resp = StopAppResponse()
        
        resp.stopped, resp.message,subscribers,publishers,services = self.apps['from_source'][req.name].stop() 

        self.flips(subscribers,ConnectionType.SUBSCRIBER,False)
        self.flips(publishers,ConnectionType.PUBLISHER,False)
        self.flips(services,ConnectionType.SERVICE,False)

        return resp

    def process_stdmsg(self,message):
        self.pubs['log'].publish(message)

#    def pullinTopics(self,topics,cancel_flag):

    def flips(self,topics,type,ok_flag):

        if len(topics) == 0:
            return
        req = RemoteRequest()
        req.cancel = not ok_flag

        req.remotes = []

        for t in topics:
            req.remotes.append(self.createRemoteRule(self.param['remote_name'],self.createRule(t,type)))

        resp = self.gateway_srv['flip'](req)

        if resp.result == 0:
            rospy.loginfo("Succuess to Flip")
        else:
            rospy.logerr( "Fail to Flip     : %s"%resp.error_message)


    def spin(self):
        # TODO: Test if gateway is connected
        self.makePublic(True)
        rospy.spin()
        self.makePublic(False)


