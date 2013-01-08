#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_appmanager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import os
import sys
import traceback
import roslaunch.pmon
from .app import App
from .app_list import AppListFile
from .logger import Logger
from std_msgs.msg import String
from .utils import platform_compatible, platform_tuple
import appmanager_msgs.msg as appmanager_msgs
import appmanager_msgs.srv as appmanager_srvs
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs

##############################################################################
# App Manager
##############################################################################


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


class AppManager(object):

    DEFAULT_APP_LIST_DIRECTORY = None # '/opt/ros/groovy/stacks/'

    init_srv_name = '~init'
    flip_request_srv_name = '~apiflip_request'
    invitation_srv_name = '~invitation'

    listapp_srv_name = 'list_apps'
    platform_info_srv_name = 'platform_info'
    start_app_srv_name = 'start_app'
    stop_app_srv_name = 'stop_app'
    log_pub_name = 'log'

    services = {}
    pubs = {}
    gateway_srvs = {}

    APP_STOPPED = "stopped"
    APP_RUNNING = "running"
    APP_BROKEN = "broken"

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self.param = {}
        self.apps = {}
        self.app_list = None

        self._setup_ros_parameters()
        self._app_status = self.APP_STOPPED

        roslaunch.pmon._init_signal_handlers()

        self._set_gateway_services()
        self._set_platform_info()

        self._get_installed_app_list()  # It sets up an app directory and load installed app list from directory
        self._set_app_manager_api()  # Must be after get app list, so it doesn't try to access nonexistant apps['from_source']

    def _set_app_manager_api(self):
        self.services = {}
        self.services['init'] = rospy.Service(self.init_srv_name, appmanager_srvs.Init, self._process_init)
        self.services['apiflip_request'] = rospy.Service(self. flip_request_srv_name, appmanager_srvs.FlipRequest, self._process_flip_request)
        self.services['invitation'] = rospy.Service(self.invitation_srv_name, concert_srvs.Invitation, self._process_invitation)

    def _set_gateway_services(self):
        self.gateway_srv = {}
        self.gateway_srv['flip'] = rospy.ServiceProxy('~flip', gateway_srvs.Remote)
        self.gateway_srv['advertise'] = rospy.ServiceProxy('~advertise', gateway_srvs.Advertise)
        self.gateway_srv['pull'] = rospy.ServiceProxy('~pull', gateway_srvs.Remote)

    def _set_platform_info(self):
        self.platform_info = concert_msgs.PlatformInfo()
        self.platform_info.platform = concert_msgs.Constants.PLATFORM_LINUX
        self.platform_info.system = concert_msgs.Constants.SYSTEM_ROS
        self.platform_info.robot = self.param['robot_type']  # TODO Validate this against concert_msgs.Constants ROBOT_XXX
        self.platform_info.name = self.param['robot_name']

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def _process_init(self, req):
        self.name = req.name
        try:
            self.setAPIs(self.name)
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return appmanager_srvs.InitResponse(False)
        return appmanager_srvs.InitResponse(True)

    def _process_flip_request(self, req):
        try:
            remotename = req.remotename
            self.remotename = remotename
            service = self.service_names[0:2]
            self.flips(remotename, service, gateway_msgs.ConnectionType.SERVICE, req.ok_flag)
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return appmanager_srvs.FlipRequestResponse(False)

        return appmanager_srvs.FlipRequestResponse(True)

    def _process_invitation(self, req):
        self.log(str(req))
        #self.remotename = req.name
        service = self.service_names[2:]
        self.log("accepted invitation to remote concert [%s]" % str(self.remotename))

        try:
            self.flips(self.remotename, service, gateway_msgs.ConnectionType.SERVICE, req.ok_flag)
            self.flips(self.remotename, self.pub_names, gateway_msgs.ConnectionType.PUBLISHER, req.ok_flag)
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return concert_srvs.InvitationResponse(False)
        return concert_srvs.InvitationResponse(True)

    def _process_platform_info(self, req):
        return concert_srvs.GetPlatformInfoResponse(self.platform_info)

    def _process_get_app_list(self, req):
        apps_description = appmanager_srvs.GetAppListResponse()

        for app_name in self.apps['from_source']:
            app = self.apps['from_source'][app_name]
            a = appmanager_msgs.AppDescription()
            a.name = app.data['name']
            a.display = app.data['display']
            a.description = app.data['description']
            a.platform = app.data['platform']
            a.status = app.data['status']
            apps_description.apps.append(a)

        return apps_description

    def _process_start_app(self, req):
        resp = appmanager_srvs.StartAppResponse()
        if self._app_status == self.APP_RUNNING:
            resp.started = False
            resp.message = "an app is already running"
            return resp

        rospy.loginfo("App Manager : starting app : " + req.name)

        resp.started, resp.message, subscribers, publishers, services = \
                self.apps['from_source'][req.name].start(self.param['robot_name'], req.remappings)

        # self.pullinTopics(pullin_topics,True)
        print subscribers
        print publishers

        self.log(self.remotename)
        if self.remotename:
            self.flips(self.remotename, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER, True)
            self.flips(self.remotename, publishers, gateway_msgs.ConnectionType.PUBLISHER, True)
            self.flips(self.remotename, services, gateway_msgs.ConnectionType.SERVICE, True)
        if resp.started:
            self._app_status = self.APP_RUNNING
        return resp

    def _process_stop_app(self, req):
        if self._app_status == self.APP_STOPPED:
            return
        rospy.loginfo("App Manager : stopping app : " + req.name)
        resp = appmanager_srvs.StopAppResponse()

        resp.stopped, resp.message, subscribers, publishers, services = self.apps['from_source'][req.name].stop()

        if self.remotename:
            self.flips(self.remotename, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER, False)
            self.flips(self.remotename, publishers, gateway_msgs.ConnectionType.PUBLISHER, False)
            self.flips(self.remotename, services, gateway_msgs.ConnectionType.SERVICE, False)
        if resp.stopped:
            self._app_status = self.APP_STOPPED
        return resp

    ##########################################################################
    # Utilities
    ##########################################################################

    def createRemoteRule(self, gateway, rule):
        r = gateway_msgs.RemoteRule()
        r.gateway = gateway
        r.rule = rule

        return r

    def createRule(self, name, api_type):
        r = gateway_msgs.Rule()
        r.name = name
        r.type = api_type
        r.node = ''
        return r

    def _setup_ros_parameters(self):
        rospy.logdebug("App Manager : parsing parameters")
        param = {}
        param['robot_type'] = rospy.get_param('~robot_type')
        param['robot_name'] = rospy.get_param('~robot_name')
        param['app_from_source_directory'] = rospy.get_param('~app_from_source_directory', self.DEFAULT_APP_LIST_DIRECTORY)
        param['app_store_url'] = rospy.get_param('~app_store_url', '')
        param['platform_info'] = rospy.get_param('~platform_info', '')
        param['white_list'] = rospy.get_param('~whitelist', '')
        param['black_list'] = rospy.get_param('~black_list', '')
        param['is_alone'] = rospy.get_param('~is_alone', False)
        param['app_lists'] = rospy.get_param('~app_lists', '').split(';')
        # If we have list parameters - https://github.com/ros/ros_comm/pull/50/commits
        # param['app_lists'] = rospy.get_param('~app_lists', [])
        self.param = param

    def _get_installed_app_list(self):
        '''
         Sets up an app directory and load installed app from given directory
        '''
        apps = {}
        apps['from_source'] = {}
        # Getting apps from installed list
        for filename in self.param['app_lists']:
            app_list_file = AppListFile(filename)
            # ach, put in jihoon's format
            for app in app_list_file.available_apps:
                if platform_compatible(platform_tuple(self.platform_info.platform, self.platform_info.system, self.platform_info.robot), app.data['platform']):
                    apps['from_source'][app.data['name']] = app
        # Getting apps from source
        directory = self.param['app_from_source_directory']
        if directory:
            for (app_name, app_file) in self._load(directory, '.app'):
                try:
                    app = App()
                    app.load_from_app_file(app_name, app_file)
                    if platform_compatible(platform_tuple(self.platform_info.platform, self.platform_info.system, self.platform_info.robot), app.data['platform']):
                        apps['from_source'][app_name] = app
                except Exception as e:
                    print str(e)
    #                del apps['from_source'][app_str[0]]
        self.apps = apps

    def _load(self, directory, typ):
        '''
          It searchs *.app in directories
        '''
        applist = []
        for dpath, unused_, files in os.walk(directory):
            apps = [f for f in files if f.endswith(typ)]
            apps_with_path = [dpath + '/' + a for a in apps]
            apps_name = [a[0:len(a) - len(typ)] for a in apps]

            applist += list(zip(apps_name, apps_with_path))

        return applist

    def setAPIs(self, namespace):
        rospy.loginfo("App Manager : advertising services")
        self.platform_info.name = namespace
        service_names = [namespace + '/' + self.listapp_srv_name, '/' + namespace + '/' + self.platform_info_srv_name, '/' + namespace + '/' + self.start_app_srv_name, '/' + namespace + '/' + self.stop_app_srv_name]
        self.service_names = service_names
        self.services['list_apps'] = rospy.Service(service_names[0], appmanager_srvs.GetAppList, self._process_get_app_list)
        self.services['platform_info'] = rospy.Service(service_names[1], concert_srvs.GetPlatformInfo, self._process_platform_info)
        self.services['start_app'] = rospy.Service(service_names[2], appmanager_srvs.StartApp, self._process_start_app)
        self.services['stop_app'] = rospy.Service(service_names[3], appmanager_srvs.StopApp, self._process_stop_app)

        self.pubs = {}
        pub_names = ['/' + namespace + '/' + self.log_pub_name]
        self.pub_names = pub_names
        self.pubs['log'] = rospy.Publisher(self.pub_names[0], String)

        # Logging mechanism. hooks std_out and publish as a topic
        self.logger = Logger(sys.stdout)
        sys.stdout = self.logger
        self.logger.addCallback(self.process_stdmsg)

    def flips(self, remotename, topics, type, ok_flag):
        if len(topics) == 0:
            return
        req = gateway_srvs.RemoteRequest()
        req.cancel = not ok_flag

        req.remotes = []

        for t in topics:
            req.remotes.append(self.createRemoteRule(remotename, self.createRule(t, type)))

        resp = self.gateway_srv['flip'](req)

        if resp.result == 0:
            rospy.loginfo("App Manager : successfully flipped %s" % str(topics))
        else:
            rospy.logerr("App Manager : failed to flip [%s]" % resp.error_message)

    def process_stdmsg(self, message):
        self.pubs['log'].publish(message)

    def log(self, msg):
        rospy.loginfo("App Manager : " + msg)

    def logerr(self, msg):
        rospy.logerr("App Manager : " + msg)

    def spin(self):
        # TODO: Test if gateway is connected
#        self.makePublic(True)
        rospy.spin()
#        self.makePublic(False)
