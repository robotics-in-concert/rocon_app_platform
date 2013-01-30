#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_app_manager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import os
import sys
import traceback
import roslaunch.pmon
from .rapp import Rapp
from .rapp_list import RappListFile
from std_msgs.msg import String
from .utils import platform_compatible, platform_tuple
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_app_manager_msgs.srv as rapp_manager_srvs
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs

##############################################################################
# App Manager
##############################################################################


"""
    RobotAppManager ~ RoconAppManager
"""


class RappManager(object):

    DEFAULT_APP_LIST_DIRECTORY = None # '/opt/ros/groovy/stacks/'

    init_srv_name = '~init'
    invitation_srv_name = '~invitation'

    log_pub_name = 'log'

    services = {}
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
        self._app_status = self.APP_STOPPED
        roslaunch.pmon._init_signal_handlers()

        self._setup_ros_parameters()
        self._set_platform_info()
        self._init_gateway_services()
        self._init_app_manager_services()

        self._get_installed_app_list()  # It sets up an app directory and load installed app list from directory

    def _set_platform_info(self):
        self.platform_info = rapp_manager_msgs.PlatformInfo()
        self.platform_info.platform = concert_msgs.Constants.PLATFORM_LINUX
        self.platform_info.system = concert_msgs.Constants.SYSTEM_ROS
        self.platform_info.robot = self.param['robot_type']  # TODO Validate this against concert_msgs.Constants ROBOT_XXX, but shouldn't be a concert_msgs type
        self.platform_info.name = self.param['robot_name']

    def _init_app_manager_services(self):
        self._service_names = {}
        self._service_names['platform_info'] = 'platform_info' # these will get prefixed by the gateway name later
        self._service_names['list_apps'] = 'list_apps'
        self._service_names['start_app'] = 'start_app'
        self._service_names['stop_app'] = 'stop_app'

        self.services = {}
        self.services['init'] = rospy.Service(self.init_srv_name, rapp_manager_srvs.Init, self._process_init)
        self.services['invitation'] = rospy.Service(self.invitation_srv_name, concert_srvs.Invitation, self._process_invitation)

    def _init_gateway_services(self):
        self.gateway_srv = {}
        self.gateway_srv['flip'] = rospy.ServiceProxy('~flip', gateway_srvs.Remote)
        self.gateway_srv['advertise'] = rospy.ServiceProxy('~advertise', gateway_srvs.Advertise)
        self.gateway_srv['pull'] = rospy.ServiceProxy('~pull', gateway_srvs.Remote)

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def _process_init(self, req):
        self.name = req.name
        # Prefix all services with the unique name
        for name in self._service_names:
            self._service_names[name] = '/' + self.name + '/' + name 
        
        try:
            self.setAPIs(self.name)
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return rapp_manager_srvs.InitResponse(False)
        return rapp_manager_srvs.InitResponse(True)

    def _process_invitation(self, req):
        self.log(str(req))
        self._remote_name = req.name
        self.log("accepted invitation to remote concert [%s]" % str(self._remote_name))

        try:
            self._flip_connections(self._remote_name, 
                                   [self._service_names['start_app'], self._service_names['stop_app']], 
                                   gateway_msgs.ConnectionType.SERVICE, 
                                   req.ok_flag
                                   )
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return concert_srvs.InvitationResponse(False)
        return concert_srvs.InvitationResponse(True)

    def _process_platform_info(self, req):
        return rapp_manager_srvs.GetPlatformInfoResponse(self.platform_info)

    def _process_get_app_list(self, req):
        apps_description = rapp_manager_srvs.GetAppListResponse()

        for app_name in self.apps['from_source']:
            app = self.apps['from_source'][app_name]
            a = rapp_manager_msgs.AppDescription()
            a.name = app.data['name']
            a.display = app.data['display']
            a.description = app.data['description']
            a.platform = app.data['platform']
            a.status = app.data['status']
            apps_description.apps.append(a)

        return apps_description

    def _process_start_app(self, req):
        resp = rapp_manager_srvs.StartAppResponse()
        if self._app_status == self.APP_RUNNING:
            resp.started = False
            resp.message = "an app is already running"
            return resp

        rospy.loginfo("App Manager : starting app : " + req.name)

        resp.started, resp.message, subscribers, publishers, services = \
                self.apps['from_source'][req.name].start(self.param['robot_name'], req.remappings)

        # self.pullinTopics(pullin_topics,True)
        #print subscribers
        #print publishers

        self.log(self._remote_name)
        if self._remote_name:
            self._flip_connections(self._remote_name, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER, True)
            self._flip_connections(self._remote_name, publishers, gateway_msgs.ConnectionType.PUBLISHER, True)
            self._flip_connections(self._remote_name, services, gateway_msgs.ConnectionType.SERVICE, True)
        if resp.started:
            self._app_status = self.APP_RUNNING
        return resp

    def _process_stop_app(self, req):
        if self._app_status == self.APP_STOPPED:
            return
        rospy.loginfo("App Manager : stopping app : " + req.name)
        resp = rapp_manager_srvs.StopAppResponse()

        resp.stopped, resp.message, subscribers, publishers, services = self.apps['from_source'][req.name].stop()

        if self._remote_name:
            self._flip_connections(self._remote_name, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER, False)
            self._flip_connections(self._remote_name, publishers, gateway_msgs.ConnectionType.PUBLISHER, False)
            self._flip_connections(self._remote_name, services, gateway_msgs.ConnectionType.SERVICE, False)
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
            # should do some exception checking here, also utilise AppListFile properly.
            app_list_file = RappListFile(filename)
            # ach, put in jihoon's format
            for app in app_list_file.available_apps:
                if platform_compatible(platform_tuple(self.platform_info.platform, self.platform_info.system, self.platform_info.robot), app.data['platform']):
                    apps['from_source'][app.data['name']] = app
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

        # To be advertised services
        self.services['platform_info'] = rospy.Service(self._service_names['platform_info'], rapp_manager_srvs.GetPlatformInfo, self._process_platform_info)
        self.services['list_apps'] = rospy.Service(self._service_names['list_apps'], rapp_manager_srvs.GetAppList, self._process_get_app_list)
        self._advertise_service(self._service_names['platform_info'])
        self._advertise_service(self._service_names['list_apps'])

        # To be flipped services
        self.services['start_app'] = rospy.Service(self._service_names['start_app'], rapp_manager_srvs.StartApp, self._process_start_app)
        self.services['stop_app'] = rospy.Service(self._service_names['stop_app'], rapp_manager_srvs.StopApp, self._process_stop_app)

    def _advertise_service(self, service_name):
        '''
          Advertise a rocon_app_manager service via the gateway.
          
          @param service_name
          @type string
        '''
        req = gateway_srvs.AdvertiseRequest()
        req.cancel = False
        req.rules = []
        rule = gateway_msgs.Rule()
        rule.type = gateway_msgs.ConnectionType.SERVICE
        rule.name = service_name
        rule.node = ''  # Shouldn't need to worry about setting the node name here, maybe rospy.get_name()
        req.rules.append(rule)
        unused_resp = self.gateway_srv['advertise'](req)
        
    def _flip_connections(self, remote_name, connection_names, connection_type, direction_flag):
        '''
          (Un)Flip a service to a remote gateway.
          
          @param remote_name : the name of the remote gateway to flip to. 
          @type str
          @param connection_names : the topic/service/action_xxx names
          @type list of str
          @param connection_type : publisher, subscriber, service, action_client or action_server
          @type gateway_msgs.ConnectionType
          @param cancel_flag : whether or not we are flipping (false) or unflipping (true)
          @type bool
        '''
        if len(connection_names) == 0:
            return
        req = gateway_srvs.RemoteRequest()
        req.cancel = not direction_flag 
        req.remotes = []
        for connection_name in connection_names:
            req.remotes.append(self.createRemoteRule(remote_name, self.createRule(connection_name, connection_type)))
        resp = self.gateway_srv['flip'](req)
        if resp.result == 0:
            rospy.loginfo("App Manager : successfully flipped %s" % str(connection_names))
        else:
            rospy.logerr("App Manager : failed to flip [%s]" % resp.error_message)

    def log(self, msg):
        rospy.loginfo("App Manager : " + msg)

    def logerr(self, msg):
        rospy.logerr("App Manager : " + msg)

    def spin(self):
        # TODO: Test if gateway is connected
#        self.makePublic(True)
        rospy.spin()
#        self.makePublic(False)
