#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/hydro-devel/rocon_app_manager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import os
import sys
import time
import thread
import traceback
import roslaunch.pmon
from .rapp_list import RappListFile
from .utils import platform_compatible, platform_tuple
import rocon_utilities
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
        Robot App Manager ~ Rocon App Manager
    """

    default_application_namespace = "application"

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self._namespace = None  # Namespace that gets used as default namespace for rapp connections
        self._gateway_name = None  # Name of our local gateway (if available)
        self._remote_name = None  # Name (gateway name) for the entity that is remote controlling this app manager
        self._current_rapp = None  # App that is running, otherwise None
        self._application_namespace = None  # Push all app connections underneath this namespace
        roslaunch.pmon._init_signal_handlers()
        self._services = {}

        self._setup_ros_parameters()
        self._set_platform_info()
        self._init_gateway_services()
        self._init_default_service_names()

        self._get_pre_installed_app_list()  # It sets up an app directory and load installed app list from directory
        self._init_services()

    def _setup_ros_parameters(self):
        rospy.logdebug("App Manager : parsing parameters")
        self._param = {}
        self._param['robot_type'] = rospy.get_param('~robot_type', 'robot')
        self._param['robot_name'] = rospy.get_param('~robot_name', 'app_manager')
        self._param['app_store_url'] = rospy.get_param('~app_store_url', '')
        self._param['platform_info'] = rospy.get_param('~platform_info', 'linux.ros.*')
        self._param['rapp_lists'] = rospy.get_param('~rapp_lists', '').split(';')
        # Todo fix these up with proper whitelist/blacklists
        self._param['remote_controller_whitelist'] = rospy.get_param('~remote_controller_whitelist', [])
        self._param['remote_controller_blacklist'] = rospy.get_param('~remote_controller_blacklist', [])

        # If we have list parameters - https://github.com/ros/ros_comm/pull/50/commits
        # self._param['rapp_lists'] = rospy.get_param('~rapp_lists', [])

    def _set_platform_info(self):
        self.platform_info = rapp_manager_msgs.PlatformInfo()
        self.platform_info.platform = rapp_manager_msgs.PlatformInfo.PLATFORM_LINUX
        self.platform_info.system = rapp_manager_msgs.PlatformInfo.SYSTEM_ROS
        self.platform_info.robot = self._param['robot_type']  # TODO Validate this against rapp_manager_msgs.PlatformInfo ROBOT_XXX
        self.platform_info.name = self._param['robot_name']

    def _init_default_service_names(self):
        self._default_service_names = {}
        self._default_service_names['platform_info'] = 'platform_info'
        self._default_service_names['list_apps'] = 'list_apps'
        self._default_service_names['status'] = 'status'
        self._default_service_names['invite'] = 'invite'
        self._default_service_names['start_app'] = 'start_app'
        self._default_service_names['stop_app'] = 'stop_app'

    def _init_gateway_services(self):
        self._gateway_services = {}
        self._gateway_services['gateway_info'] = rocon_utilities.SubscriberProxy('~gateway_info', gateway_msgs.GatewayInfo)
        self._gateway_services['flip'] = rospy.ServiceProxy('~flip', gateway_srvs.Remote)
        self._gateway_services['advertise'] = rospy.ServiceProxy('~advertise', gateway_srvs.Advertise)
        self._gateway_services['pull'] = rospy.ServiceProxy('~pull', gateway_srvs.Remote)

    def _init_services(self):
        if self._services:
            for service in self._services.values():
                service.shutdown()
            self._services = {}
        self._service_names = {}
        if self._gateway_name:
            for name in self._default_service_names:
                self._service_names[name] = '/' + self._gateway_name + '/' + name
            self._application_namespace = self._gateway_name
        else:  # It's a local standalone initialisation
            self._application_namespace = RappManager.default_application_namespace
            for name in self._default_service_names:
                self._service_names[name] = '~' + name
        try:
            # Advertisable services - we advertise these by default advertisement rules for the app manager's gateway.
            self._services['platform_info'] = rospy.Service(self._service_names['platform_info'], rapp_manager_srvs.GetPlatformInfo, self._process_platform_info)
            self._services['list_apps'] = rospy.Service(self._service_names['list_apps'], rapp_manager_srvs.GetAppList, self._process_get_app_list)
            self._services['status'] = rospy.Service(self._service_names['status'], rapp_manager_srvs.Status, self._process_status)
            self._services['invite'] = rospy.Service(self._service_names['invite'], rapp_manager_srvs.Invite, self._process_invite)
            # Flippable services
            self._services['start_app'] = rospy.Service(self._service_names['start_app'], rapp_manager_srvs.StartApp, self._process_start_app)
            self._services['stop_app'] = rospy.Service(self._service_names['stop_app'], rapp_manager_srvs.StopApp, self._process_stop_app)
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return rapp_manager_srvs.InitResponse(False)
        return rapp_manager_srvs.InitResponse(True)

    def _get_pre_installed_app_list(self):
        '''
         Retrieves app lists from yaml file.
        '''
        self.apps = {}
        self.apps['pre_installed'] = {}
        # Getting apps from installed list
        for filename in self._param['rapp_lists']:
            # should do some exception checking here, also utilise AppListFile properly.
            try:
                app_list_file = RappListFile(filename)
            except IOError as e:  # if file is not found
                rospy.logwarn("App Manager : %s" % str(e))
                return
            for app in app_list_file.available_apps:
                if platform_compatible(platform_tuple(self.platform_info.platform, self.platform_info.system, self.platform_info.robot), app.data['platform']):
                    self.apps['pre_installed'][app.data['name']] = app
                else:
                    rospy.logwarn('App : ' + str(app.data['name']) + ' is incompatible. App : (' + str(app.data['platform']) + ')  System : (' +
                                  str(self.platform_info.platform) + '.' + str(self.platform_info.system) + '.' + str(self.platform_info.robot) + ')')

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
        #rospy.loginfo("App Manager : " + str(req))
        self._remote_name = req.remote_target_name
        self._application_namespace = RappManager.default_application_namespace if req.application_namespace == '' else req.application_namespace
        rospy.loginfo("App Manager : accepting invitation to relay controls to remote system [%s]" % str(self._remote_name))
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
        response.app_status = rapp_manager_msgs.Constants.APP_RUNNING if self._current_rapp \
                         else rapp_manager_msgs.Constants.APP_STOPPED
        if self._remote_name:
            response.remote_controller = self._remote_name
        else:
            response.remote_controller = rapp_manager_msgs.Constants.NO_REMOTE_CONNECTION
        response.application_namespace = self._application_namespace
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
        resp.app_namespace = self._application_namespace
        if self._current_rapp:
            resp.started = False
            resp.message = "an app is already running"
            return resp

        rospy.loginfo("App Manager : starting app : " + req.name)

        rapp = self.apps['pre_installed'][req.name]
        resp.started, resp.message, subscribers, publishers, services, action_clients, action_servers = \
                        rapp.start(self._application_namespace, req.remappings)

        rospy.loginfo("App Manager : %s" % self._remote_name)
        if self._remote_name:
            self._flip_connections(self._remote_name, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER)
            self._flip_connections(self._remote_name, publishers, gateway_msgs.ConnectionType.PUBLISHER)
            self._flip_connections(self._remote_name, services, gateway_msgs.ConnectionType.SERVICE)
            self._flip_connections(self._remote_name, action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT)
            self._flip_connections(self._remote_name, action_servers, gateway_msgs.ConnectionType.ACTION_SERVER)
        if resp.started:
            self._current_rapp = rapp
            thread.start_new_thread(self._monitor_rapp, ())
        return resp

    def _process_stop_app(self, req=None):
        '''
          Stops a currently running rapp. This can be triggered via the stop_app service call (in which
          case req is configured), or if the rapp monitoring thread detects that it has
          naturally stopped by itself (in which case req is None).

          @param req : variable configured when triggered from the service call.
        '''
        resp = rapp_manager_srvs.StopAppResponse()
        if not self._current_rapp:
            resp.stopped = False
            resp.error_code = rapp_manager_msgs.ErrorCodes.RAPP_IS_NOT_RUNNING
            resp.message = "tried to stop a rapp, but no rapp found running"
            rospy.logwarn("App Manager : received a request to stop a rapp, but no rapp found running.")
            return resp

        rospy.loginfo("App Manager : stopping rapp : " + self._current_rapp.data['name'])

        resp.stopped, resp.message, subscribers, publishers, services, action_clients, action_servers = \
                self._current_rapp.stop()

        if self._remote_name:
            self._flip_connections(self._remote_name, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER, True)
            self._flip_connections(self._remote_name, publishers, gateway_msgs.ConnectionType.PUBLISHER, True)
            self._flip_connections(self._remote_name, services, gateway_msgs.ConnectionType.SERVICE, True)
            self._flip_connections(self._remote_name, action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT, True)
            self._flip_connections(self._remote_name, action_servers, gateway_msgs.ConnectionType.ACTION_SERVER, True)
        if resp.stopped:
            self._current_rapp = None
        return resp

    ##########################################################################
    # Utilities
    ##########################################################################

    def _monitor_rapp(self):
        '''
         Monitors an executing rapp's status to determine if it's finished
         yet or not.Move this to the rapp_manager and pass it in via the app_monitor variable
         in the constructor.

         https://github.com/robotics-in-concert/rocon_app_platform/issues/31
        '''
        while self._current_rapp:  # can be unset if stop_app service was directly called
            if not self._current_rapp.is_running():
                self._process_stop_app()
                break
            time.sleep(0.1)

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
          Advertise rocon_app_manager services via the gateway,
          if it is available.

          @param service_names
          @type string
        '''
        if self._gateway_name:
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
        try:
            resp = self._gateway_services['flip'](req)
        except rospy.service.ServiceException:
            # often disappears when the gateway shuts down just before the app manager, ignore silently.
            return
        if resp.result == 0:
            rospy.loginfo("App Manager : successfully flipped %s" % str([os.path.basename(name) for name in connection_names]))
        else:
            rospy.logerr("App Manager : failed to flip [%s]" % resp.error_message)

    def spin(self):
        while not rospy.is_shutdown():
            gateway_info = self._gateway_services['gateway_info'](timeout=rospy.Duration(0.3))
            if gateway_info:
                if gateway_info.connected:
                    self._gateway_name = gateway_info.name
                    self._init_services()
                    break
            # don't need a sleep since our timeout on the service call acts like this.
        rospy.spin()
