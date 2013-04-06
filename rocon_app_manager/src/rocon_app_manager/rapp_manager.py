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
from .rapp_list import RappListFile
from .utils import platform_compatible, platform_tuple
from rocon_utilities import create_gateway_rule, create_gateway_remote_rule
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_app_manager_msgs.srv as rapp_manager_srvs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs

##############################################################################
# App Manager
##############################################################################


class RappManager(object):
    """
        RobotAppManager ~ RoconAppManager
    """

    init_srv_name = '~init'

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self._namespace = "app_manager"
        self._app_status = rapp_manager_msgs.Constants.APP_STOPPED
        self._remote_name = None
        roslaunch.pmon._init_signal_handlers()

        self._setup_ros_parameters()
        self._set_platform_info()
        self._init_gateway_services()
        self._init_app_manager_services()

        self._get_pre_installed_app_list()  # It sets up an app directory and load installed app list from directory

    def _setup_ros_parameters(self):
        rospy.logdebug("App Manager : parsing parameters")
        self._param = {}
        self._param['robot_type'] = rospy.get_param('~robot_type')
        self._param['robot_name'] = rospy.get_param('~robot_name')
#        self._namespace = self._param['robot_name']
        self._param['app_store_url'] = rospy.get_param('~app_store_url', '')
        self._param['platform_info'] = rospy.get_param('~platform_info', '')
        self._param['app_lists'] = rospy.get_param('~app_lists', '').split(';')
        # Todo fix these up with proper whitelist/blacklists
        self._param['remote_controller_whitelist'] = rospy.get_param('~remote_controller_whitelist', [])
        self._param['remote_controller_blacklist'] = rospy.get_param('~remote_controller_blacklist', [])

        # If we have list parameters - https://github.com/ros/ros_comm/pull/50/commits
        # self._param['app_lists'] = rospy.get_param('~app_lists', [])

    def _set_platform_info(self):
        self.platform_info = rapp_manager_msgs.PlatformInfo()
        self.platform_info.platform = rapp_manager_msgs.PlatformInfo.PLATFORM_LINUX
        self.platform_info.system = rapp_manager_msgs.PlatformInfo.SYSTEM_ROS
        self.platform_info.robot = self._param['robot_type']  # TODO Validate this against rapp_manager_msgs.PlatformInfo ROBOT_XXX
        self.platform_info.name = self._param['robot_name']

    def _init_app_manager_services(self):
        self._service_names = {}
        self._service_names['platform_info'] = 'platform_info'
        self._service_names['list_apps'] = 'list_apps'
        self._service_names['status'] = 'status'
        self._service_names['invite'] = 'invite'
        self._service_names['start_app'] = 'start_app'
        self._service_names['stop_app'] = 'stop_app'

        self._services = {}

        # Other services currently only fire up when the app manager gets initialised
        # with a remote target name later. Might be nice to fire them up by default,
        # and then close them, restart them when re-initialised with a different remote
        # target later.

    def _init_gateway_services(self):
        self._gateway_services = {}
        self._gateway_services['gateway_info'] = rospy.ServiceProxy('~gateway_info', gateway_srvs.GatewayInfo)
        self._gateway_services['flip'] = rospy.ServiceProxy('~flip', gateway_srvs.Remote)
        self._gateway_services['advertise'] = rospy.ServiceProxy('~advertise', gateway_srvs.Advertise)
        self._gateway_services['pull'] = rospy.ServiceProxy('~pull', gateway_srvs.Remote)

    def _get_pre_installed_app_list(self):
        '''
         Retrieves app lists from yaml file.
        '''
        self.apps = {}
        self.apps['pre_installed'] = {}
        # Getting apps from installed list
        for filename in self._param['app_lists']:
            # should do some exception checking here, also utilise AppListFile properly.
            app_list_file = RappListFile(filename)
            # ach, put in jihoon's format
            for app in app_list_file.available_apps:
                if platform_compatible(platform_tuple(self.platform_info.platform, self.platform_info.system, self.platform_info.robot), app.data['platform']):
                    self.apps['pre_installed'][app.data['name']] = app
                else:
                    rospy.logwarn('App : ' + str(app.data['name']) +' is incompatible. App : (' + str(app.data['platform']) +')  System : (' + str(self.platform_info.platform) + '.'+str(self.platform_info.system)+'.'+str(self.platform_info.robot)+')')



    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def _process_invite(self, req):
        # Todo : add checks for whether client is currently busy or not
        if req.remote_target_name in self._param['remote_controller_whitelist']:
            return rapp_manager_srvs.InviteResponse(self._accept_invitation(req))
        elif len(self._param['remote_controller_whitelist']) == 0 and req.remote_target_name not in self._param['remote_controller_blacklist']:
            return rapp_manager_srvs.InviteResponse(self._accept_invitation(req))
        else:
            return rapp_manager_srvs.InviteResponse(False)

    def _accept_invitation(self, req):
        rospy.loginfo("App Manager : " + str(req))
        self._remote_name = req.remote_target_name
        rospy.loginfo("App Manager : accepting invitation to remote concert [%s]" % str(self._remote_name))
        try:
            self._flip_connections(self._remote_name,
                                   [self._service_names['start_app'], self._service_names['stop_app']],
                                   gateway_msgs.ConnectionType.SERVICE,
                                   req.cancel
                                   )
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return False
        return True

    def _process_platform_info(self, req):
        return rapp_manager_srvs.GetPlatformInfoResponse(self.platform_info)

    def _process_status(self, req):
        '''
          Serve some details about the current app manager status:

          - who is controlling it (i.e. who it flipped start_app etc to)
          - the namespace it is publishing it and its apps interfaces on
          - the current app status (runnning or stopped)

          @param req : status request object (empty)
          @type rapp_maanger_srvs.StatusRequest
        '''
        response = rapp_manager_srvs.StatusResponse()
        response.app_status = self._app_status
        if self._remote_name:
            response.remote_controller = self._remote_name
        else:
            response.remote_controller = rapp_manager_msgs.Constants.NO_REMOTE_CONNECTION
        response.namespace = self._param['robot_name']
        return response

    def _process_get_app_list(self, req):
        apps_description = rapp_manager_srvs.GetAppListResponse()

        for app_name in self.apps['pre_installed']:
            app = self.apps['pre_installed'][app_name]
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
        if self._app_status == rapp_manager_msgs.Constants.APP_RUNNING:
            resp.started = False
            resp.message = "an app is already running"
            return resp

        rospy.loginfo("App Manager : starting app : " + req.name)

        resp.started, resp.message, subscribers, publishers, services, action_clients, action_servers = \
                self.apps['pre_installed'][req.name].start(self._param['robot_name'], req.remappings)

        rospy.loginfo("App Manager : %s" % self._remote_name)
        if self._remote_name:
            self._flip_connections(self._remote_name, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER)
            self._flip_connections(self._remote_name, publishers, gateway_msgs.ConnectionType.PUBLISHER)
            self._flip_connections(self._remote_name, services, gateway_msgs.ConnectionType.SERVICE)
            self._flip_connections(self._remote_name, action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT)
            self._flip_connections(self._remote_name, action_servers, gateway_msgs.ConnectionType.ACTION_SERVER)
        if resp.started:
            self._app_status = rapp_manager_msgs.Constants.APP_RUNNING
        return resp

    def _process_stop_app(self, req):
        if self._app_status == rapp_manager_msgs.Constants.APP_STOPPED:
            return
        rospy.loginfo("App Manager : stopping app : " + req.name)
        resp = rapp_manager_srvs.StopAppResponse()

        resp.stopped, resp.message, subscribers, publishers, services, action_clients, action_servers = \
                self.apps['pre_installed'][req.name].stop()

        if self._remote_name:
            self._flip_connections(self._remote_name, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER, True)
            self._flip_connections(self._remote_name, publishers, gateway_msgs.ConnectionType.PUBLISHER, True)
            self._flip_connections(self._remote_name, services, gateway_msgs.ConnectionType.SERVICE, True)
            self._flip_connections(self._remote_name, action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT, True)
            self._flip_connections(self._remote_name, action_servers, gateway_msgs.ConnectionType.ACTION_SERVER, True)
        if resp.stopped:
            self._app_status = rapp_manager_msgs.Constants.APP_STOPPED
        return resp

    ##########################################################################
    # Utilities
    ##########################################################################

    def _init_namespace(self, name):
        self._param['robot_name'] = name
        # Prefix all services with the unique name
        for name in self._service_names:
            self._service_names[name] = '/' + self._param['robot_name'] + '/' + name

        try:
            rospy.loginfo("App Manager : advertising services")
            self.platform_info.name = self._param['robot_name']

            # To be advertised services
            self._services['platform_info'] = rospy.Service(self._service_names['platform_info'], rapp_manager_srvs.GetPlatformInfo, self._process_platform_info)
            self._services['list_apps'] = rospy.Service(self._service_names['list_apps'], rapp_manager_srvs.GetAppList, self._process_get_app_list)
            self._services['status'] = rospy.Service(self._service_names['status'], rapp_manager_srvs.Status, self._process_status)
            self._services['invite'] = rospy.Service(self._service_names['invite'], rapp_manager_srvs.Invite, self._process_invite)
            self._advertise_services([
                         self._service_names['platform_info'],
                         self._service_names['list_apps'],
                         self._service_names['status'],
                         self._service_names['invite']
                         ])

            # To be flipped services
            self._services['start_app'] = rospy.Service(self._service_names['start_app'], rapp_manager_srvs.StartApp, self._process_start_app)
            self._services['stop_app'] = rospy.Service(self._service_names['stop_app'], rapp_manager_srvs.StopApp, self._process_stop_app)
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return rapp_manager_srvs.InitResponse(False)
        return rapp_manager_srvs.InitResponse(True)

    def _load(self, directory, typ):
        '''
          It searchs *.rapp in directories
        '''
        applist = []
        for dpath, unused_, files in os.walk(directory):
            apps = [f for f in files if f.endswith(typ)]
            apps_with_path = [dpath + '/' + a for a in apps]
            apps_name = [a[0:len(a) - len(typ)] for a in apps]

            applist += list(zip(apps_name, apps_with_path))

        return applist

    def _advertise_services(self, service_names):
        '''
          Advertise rocon_app_manager services via the gateway.

          @param service_name
          @type string
        '''
        req = gateway_srvs.AdvertiseRequest()
        req.cancel = False
        req.rules = []
        for service_name in service_names:
            req.rules.append(create_gateway_rule(service_name, gateway_msgs.ConnectionType.SERVICE))
        unused_resp = self._gateway_services['advertise'](req)

    def _flip_connections(self, remote_name, connection_names, connection_type, cancel_flag=False):
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
        req.cancel = cancel_flag
        req.remotes = []
        for connection_name in connection_names:
            req.remotes.append(create_gateway_remote_rule(remote_name, create_gateway_rule(connection_name, connection_type)))
        resp = self._gateway_services['flip'](req)
        if resp.result == 0:
            rospy.loginfo("App Manager : successfully flipped %s" % str(connection_names))
        else:
            rospy.logerr("App Manager : failed to flip [%s]" % resp.error_message)

    def spin(self):
        gateway_info = gateway_srvs.GatewayInfoResponse()
        gateway_info.connected = False
        while not rospy.is_shutdown() and not gateway_info.connected:
            gateway_info = self._gateway_services['gateway_info']()
            if gateway_info.connected:
                self._init_namespace(gateway_info.name)
            #else:
            #    rospy.loginfo("App Manager : gateway not yet connected,")
            rospy.sleep(1.0)
        rospy.spin()
