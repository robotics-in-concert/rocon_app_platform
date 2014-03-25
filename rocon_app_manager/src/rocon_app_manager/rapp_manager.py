#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rosgraph
import os
import sys
import time
import thread
import traceback
import roslaunch.pmon
from .caps_list import CapsList, start_capabilities_for_rapp, stop_capabilities_for_rapp
import rocon_python_comms
from rocon_gateway_utils import create_gateway_rule, create_gateway_remote_rule
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_app_manager_msgs.srv as rapp_manager_srvs
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_std_msgs.srv as rocon_std_srvs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import std_msgs.msg as std_msgs
import threading
import rocon_uri
import rospkg.os_detect
import rocon_python_utils
import rocon_app_utilities 

# local imports
from . import exceptions
from ros_parameters import setup_ros_parameters
from .rapp import Rapp, convert_rapps_from_rapp_specs
from .utils import *
##############################################################################
# App Manager
##############################################################################


class RappManager(object):
    '''
        Robot App Manager ~ Rocon App Manager
    '''

    ##########################################################################
    # Initialisation
    ##########################################################################
    def __init__(self):
        self._namespace = None  # Namespace that gets used as default namespace for rapp connections
        self._gateway_name = None  # Name of our local gateway (if available)
        self._gateway_ip = None  # IP/Hostname of our local gateway if available
        self._remote_name = None  # Name (gateway name) for the entity that is remote controlling this app manager
        self._current_rapp = None  # App that is running, otherwise None
        self._application_namespace = None  # Push all app connections underneath this namespace
        roslaunch.pmon._init_signal_handlers()
        self._services = {}
        self._publishers = {}
        self._rospack = rospkg.RosPack()  # used to speed up searches for resources

        self._param = setup_ros_parameters()
        (self._rocon_uri, self._icon) = self._set_platform_info()
        self._init_gateway_services()
        self._init_default_service_names()

        self.apps = {}
        self.app_list_file = {}
        self.caps_list = {}
        self._initialising_services = False
        self._indexer = rocon_app_utilities.RappIndexer(package_whitelist=self._param['rapp_package_whitelist'], package_blacklist=self._param['rapp_package_blacklist'])
        self._runnable_apps, self._platform_filtered_apps, self._capabilities_filtered_apps = self._determine_runnable_rapps()
        self._init_services()
        self._private_publishers = self._init_private_publishers()


        # Publish currently available rapp list
        self._publish_app_list()
                                                                                                                                  
        # Publish unavailable rapp list
        # TODO: Currently blacklisted apps and non-whitelisted apps are not provided yet
        self._publishers['incompatible_app_list'].publish([], [], self._platform_filtered_apps, self._capabilities_filtered_apps)

        if self._param['auto_start_rapp']:  # None and '' are both false here
            request = rapp_manager_srvs.StartAppRequest(self._param['auto_start_rapp'], [])
            unused_response = self._process_start_app(request)

        self._debug_ignores = {}  # a remote_controller_name : timestamp of the last time we logged an ignore response
        rospy.loginfo("App Manager : initialised.")

    def _init_private_publishers(self):
        '''
          Generate some private publishers for internal nodes to introspect on (these do not sit on the
          shifting namespace used for the remote controller and never get advertised/flipped).
        '''
        private_publishers = {}
        # this might be worth upgrading to a rocon_app_manager_msgs.srv.Status-like publisher at some point in the future
        private_publishers['remote_controller'] = rospy.Publisher('~remote_controller', std_msgs.String, latch=True)
        # initialise some of the bastards
        private_publishers['remote_controller'].publish(std_msgs.String(rapp_manager_msgs.Constants.NO_REMOTE_CONTROLLER))
        return private_publishers

    def _set_platform_info(self):
        '''
          Initialises the rapp manager with the appropriate platform info.
          This is part of the __init__ process.
        '''
        # This might be naive and only work well on ubuntu...
        os_codename = rospkg.os_detect.OsDetect().get_codename()
        rocon_uri = "rocon:/" + self._param['robot_type'] + \
                          "/" + self._param['robot_name'] + \
                          "/" + rocon_python_utils.ros.get_rosdistro() + \
                          "/" + os_codename
        try:
            filename = rocon_python_utils.ros.find_resource_from_string(self._param['robot_icon'])
            icon = rocon_python_utils.ros.icon_to_msg(filename)
        except exceptions.NotFoundException:
            rospy.logwarn("App Manager : icon resource not found [%s]" % self._param['robot_icon'])
            icon = rocon_std_msgs.Icon()
        except ValueError:
            rospy.logwarn("App Manager : invalid resource name [%s]" % self._param['robot_icon'])
            icon = rocon_std_msgs.Icon()
        return (rocon_uri, icon)

    def _init_default_service_names(self):
        self._default_service_names = {}
        self._default_service_names['platform_info'] = 'platform_info'
        self._default_service_names['list_apps'] = 'list_apps'
        self._default_service_names['status'] = 'status'
        self._default_service_names['invite'] = 'invite'
        self._default_service_names['start_app'] = 'start_app'
        self._default_service_names['stop_app'] = 'stop_app'
        # Latched publishers
        self._default_publisher_names = {}
        self._default_publisher_names['app_list'] = 'app_list'
        self._default_publisher_names['incompatible_app_list'] = 'incompatible_app_list'

    def _init_gateway_services(self):
        self._gateway_services = {}
        self._gateway_services['gateway_info'] = rocon_python_comms.SubscriberProxy('~gateway_info', gateway_msgs.GatewayInfo)
        self._gateway_services['remote_gateway_info'] = rospy.ServiceProxy('~remote_gateway_info', gateway_srvs.RemoteGatewayInfo)
        self._gateway_services['flip'] = rospy.ServiceProxy('~flip', gateway_srvs.Remote)
        self._gateway_services['advertise'] = rospy.ServiceProxy('~advertise', gateway_srvs.Advertise)
        self._gateway_services['pull'] = rospy.ServiceProxy('~pull', gateway_srvs.Remote)
        self._gateway_publishers = {}
        self._gateway_publishers['force_update'] = rospy.Publisher("~force_update", std_msgs.Empty)

    def _init_services(self):
        '''
          This initialises all the app manager services. It depends on whether we're initialising for standalone,
          or connected (pairing/concert) modes. This should not be activated multiply!
        '''
        if self._initialising_services:
            # We could use a lock to protect this, but since the only places we call this is in the
            # and in the spin(), then we just use a flag to protect.
            return False
        self._initialising_services = True
        if self._services:
            for service in self._services.values():
                service.shutdown()
            for publisher in self._publishers.values():
                publisher.unregister()
            self._services = {}
            self._publishers = {}
        self._service_names = {}
        self._publisher_names = {}
        base_name = self._gateway_name if self._gateway_name else self._param['robot_name']  # latter option is for standalone mode
        for name in self._default_service_names:
            self._service_names[name] = '/' + base_name + '/' + self._default_service_names[name]
        for name in self._default_publisher_names:
            self._publisher_names[name] = '/' + base_name + '/' + self._default_publisher_names[name]
        self._application_namespace = base_name
        try:
            # Advertisable services - we advertise these by default advertisement rules for the app manager's gateway.
            self._services['platform_info'] = rospy.Service(self._service_names['platform_info'], rocon_std_srvs.GetPlatformInfo, self._process_platform_info)
            self._services['list_apps'] = rospy.Service(self._service_names['list_apps'], rapp_manager_srvs.GetAppList, self._process_get_runnable_app_list)
            self._services['status'] = rospy.Service(self._service_names['status'], rapp_manager_srvs.Status, self._process_status)
            self._services['invite'] = rospy.Service(self._service_names['invite'], rapp_manager_srvs.Invite, self._process_invite)
            # Flippable services
            self._services['start_app'] = rospy.Service(self._service_names['start_app'], rapp_manager_srvs.StartApp, self._process_start_app)
            self._services['stop_app'] = rospy.Service(self._service_names['stop_app'], rapp_manager_srvs.StopApp, self._process_stop_app)
            # Latched publishers
            self._publishers['app_list'] = rospy.Publisher(self._publisher_names['app_list'], rapp_manager_msgs.AppList, latch=True)
            self._publishers['incompatible_app_list'] = rospy.Publisher(self._publisher_names['incompatible_app_list'], rapp_manager_msgs.IncompatibleAppList, latch=True)
            # Force an update on the gateway
            self._gateway_publishers['force_update'].publish(std_msgs.Empty())
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            self._initialising_services = False
            return False
        self._publish_app_list()  # get the latched publisher refiring
        self._initialising_services = False
        return True

    def _determine_runnable_rapps(self):
        '''
         Prune unsupported apps due to lack of support for required capabilities.

         :returns: incompatible app list dictionaries for capability incompatibilities respectively
         :rtype: {rocon_app_manager.Rapp}, [str], [str] 
        '''
        compatible_rapplist, incompatible_rapplist = self._indexer.get_compatible_rapps(self._rocon_uri)
        runnable_rapp_specs, capabilities_incompatible_rapps = self._filter_capability_unavailable_rapps(compatible_rapplist)
        runnable_rapps, defected_rapps = convert_rapps_from_rapp_specs(runnable_rapp_specs, self._rospack)

        # Log out the rapps
        for rapp in incompatible_rapplist: 
            rospy.logwarn("App : '" + str(rapp.ancestor_name) + "' is incompatible [" + rapp.data['compatibility'] + "][" + self._rocon_uri + "]")

        for rapp_name, reason in capabilities_incompatible_rapps.items():
            rospy.logwarn("App : '" + rapp_name + "' " + str(reason))

        for rapp_name in runnable_rapps.keys():
            rospy.loginfo("App : '" + rapp_name + "' added to the list of runnable apps.")

        platform_filtered_rapps = [rapp.ancestor_name for rapp in incompatible_rapplist]
        capabilities_filtered_rapps = capabilities_incompatible_rapps.keys() 

        return (runnable_rapps, platform_filtered_rapps, capabilities_filtered_rapps)

    def _filter_capability_unavailable_rapps(self, compatible_rapplist):
        '''
          Filters out rapps which does not meet the platform's capability

          :params compatible_rapplist: Platform compatible rapp dict
          :type compatible_rapplist: dict

          :returns: runnable rapp, capability filtered rapp
          :rtype: dict, dict
        '''
        is_caps_available = self._init_capabilities()  # First try initialise the list of available capabilities
        capabilities_filtered_apps = {}
        runnable_apps = {}

        # Then add runable apps to list
        for rapp in compatible_rapplist:
            if not is_caps_available:
                if 'required_capabilities' in rapp.data:
                    reason = "cannot be run, since capabilities are not available. Rapp will be excluded from the list of runnable apps."
                    capabilities_filtered_apps[rapp.ancestor_name] = reason
                else:
                    runnable_apps[rapp.ancestor_name] = rapp
            else:
                try:
                    self.caps_list.compatibility_check(rapp)
                    runnable_apps[rapp.ancestor_name] = rapp
                except exceptions.MissingCapabilitiesException as e:
                    reason = "cannot be run, since some required capabilities (" + str(e.missing_caps) + ") are not installed. App will be excluded from the list of runnable apps."
                    capabilities_filtered_apps[rapp.ancestor_name] = reason
        return runnable_apps, capabilities_filtered_apps

    def _init_capabilities(self):
        try:
            self.caps_list = CapsList()
        except exceptions.NotFoundException as e:
            if 'Timed out' in str(e) or "Couldn't find" in str(e):
                rospy.loginfo("App Manager : disabling apps requiring capabilities [%s]" % str(e))
            else:
                rospy.logwarn("App Manager : disabling apps requiring capabilities [%s]" % str(e))
            return False
        return True

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def _process_invite(self, req):
        '''
          :todo: This needs better arranging for logic. Currently it ignores whitelists/blacklists if the local_remote_controllers
          only is flagged. Not an urgent use case though.

          To fix, just do abort checks for local remote controllers first, then put it through the rest of the logic as well.

          :param req: Invitation request
          :type req: rapp_manager_srvs.InviteRequest

          :returns: Response
          :rtype: rapp_manager_srvs.InviteResponse
        '''
        # Todo : add checks for whether client is currently busy or not
        response = rapp_manager_srvs.InviteResponse(True, rapp_manager_msgs.ErrorCodes.SUCCESS, "")
        if self._param['local_remote_controllers_only'] and not req.cancel:
            # Don't run this code if cancelling - sometimes our hub will have disappeared and we
            # just want to cancel regardless. Not the cleanest exit, but it will do.
            if self._gateway_name is None:
                return rapp_manager_srvs.InviteResponse(False,
                                                        rapp_manager_msgs.ErrorCodes.NO_LOCAL_GATEWAY,
                                                        "no gateway connection yet, invite impossible.")
            remote_gateway_info_request = gateway_srvs.RemoteGatewayInfoRequest()
            remote_gateway_info_request.gateways = []
            remote_gateway_info_response = self._gateway_services['remote_gateway_info'](remote_gateway_info_request)
            remote_target_name = req.remote_target_name
            remote_target_ip = None
            for gateway in remote_gateway_info_response.gateways:
                if gateway.name == remote_target_name:
                    remote_target_ip = gateway.ip
                    break
            if remote_target_ip is not None and self._gateway_ip == remote_target_ip:
                response = self._accept_invitation(req)
            elif remote_target_ip is not None and rosgraph.network.is_local_address(remote_target_ip):
                response = self._accept_invitation(req)
            else:
                return rapp_manager_srvs.InviteResponse(False,
                                 rapp_manager_msgs.ErrorCodes.LOCAL_INVITATIONS_ONLY,
                                 "local invitations only permitted.")
        elif req.remote_target_name in self._param['remote_controller_whitelist']:
            response = self._accept_invitation(req)
        elif len(self._param['remote_controller_whitelist']) == 0 and req.remote_target_name not in self._param['remote_controller_blacklist']:
            response = self._accept_invitation(req)
        else:
            # If we get here, we are not permitted to accept, either not in the whitelist, or in the blacklist
            if len(self._param['remote_controller_whitelist']) != 0:
                response = rapp_manager_srvs.InviteResponse(False, rapp_manager_msgs.ErrorCodes.INVITING_CONTROLLER_NOT_WHITELISTED, "not flagged in a non-empty whitelist")
            else:
                response = rapp_manager_srvs.InviteResponse(False, rapp_manager_msgs.ErrorCodes.INVITING_CONTROLLER_BLACKLISTED, "this remote controller has been blacklisted")
        # publish an update locally
        if response.result:
            remote_controller = rapp_manager_msgs.Constants.NO_REMOTE_CONTROLLER if req.cancel else req.remote_target_name
            self._private_publishers['remote_controller'].publish(std_msgs.String(remote_controller))
        return response

    def _accept_invitation(self, req):
        '''
          :param req: request
          :type req: rapp_manager_srvs.InviteRequest

          :returns: response message for the invitation
          :rtype: rapp_manager_srvs.InviteResponse(result, error_code, message)
        '''
        # Abort checks
        if req.cancel and (req.remote_target_name != self._remote_name):
            rospy.logwarn("App Manager : ignoring request from %s to cancel invitation as it is not the current remote controller [%s]" % (str(req.remote_target_name), self._remote_name))
            return rapp_manager_srvs.InviteResponse(False, rapp_manager_msgs.ErrorCodes.NOT_CURRENT_REMOTE_CONTROLLER, "ignoring request from %s to cancel invitation as it is not the current remote controller [%s]")
        if not req.cancel and self._remote_name is not None:
            if req.remote_target_name == self._remote_name:
                rospy.logwarn("App Manager : bastards are sending us repeat invites, so we ignore - we are already working for them! [%s]" % self._remote_name)
                return rapp_manager_srvs.InviteResponse(True, rapp_manager_msgs.ErrorCodes.SUCCESS, "you are a bastard, stop spamming us - you already invited this app manager!")
            else:
                if (req.remote_target_name not in self._debug_ignores or
                     ((rospy.Time.now() - self._debug_ignores[req.remote_target_name]) > rospy.Duration(120))):
                        self._debug_ignores[req.remote_target_name] = rospy.Time.now()
                        rospy.loginfo("App Manager : ignoring invitation from %s [already invited by %s]" % (str(req.remote_target_name), self._remote_name))
                return rapp_manager_srvs.InviteResponse(False, rapp_manager_msgs.ErrorCodes.ALREADY_REMOTE_CONTROLLED, "already remote controlled from %s" % self._remote_name)
        # Variable setting
        if req.application_namespace == '':
            self._application_namespace = self._gateway_name if self._gateway_name else self._param['robot_name']
        else:
            self._application_namespace = req.application_namespace
        # Flips/Unflips
        try:
            self._flip_connections(req.remote_target_name,
                                   [self._service_names['start_app'], self._service_names['stop_app']],
                                   gateway_msgs.ConnectionType.SERVICE,
                                   req.cancel
                                   )
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return False
        # Cleaning up and setting final state
        if req.cancel:
            if req.remote_target_name == self._remote_name:
                rospy.loginfo("App Manager : cancelling remote control of this system [%s]" % str(req.remote_target_name))
                if self._current_rapp:
                    thread = threading.Thread(target=self._process_stop_app)
                    thread.start()
                self._remote_name = None
        else:
            rospy.loginfo("App Manager : accepting invitation to be remote controlled [%s]" % str(req.remote_target_name))
            self._remote_name = req.remote_target_name
        return rapp_manager_srvs.InviteResponse(True, rapp_manager_msgs.ErrorCodes.SUCCESS, "success")

    def _process_platform_info(self, req):
        platform_info = rocon_std_msgs.PlatformInfo(
                                version=rocon_std_msgs.Strings.ROCON_VERSION,
                                uri=self._rocon_uri,
                                icon=self._icon)
        return rocon_std_srvs.GetPlatformInfoResponse(platform_info)

    def _process_status(self, req):
        '''
          Serve some details about the current app manager status:

          - who is controlling it (i.e. who it flipped start_app etc to)
          - the namespace it is publishing it and its apps interfaces on
          - the current app status (runnning or stopped)

          :param req: status request object (empty)
          :type req: rapp_manager_srvs.StatusRequest
        '''
        response = rapp_manager_srvs.StatusResponse()
        if self._current_rapp:
            response.application_status = rapp_manager_msgs.Constants.APP_RUNNING
            response.application = self._current_rapp.to_msg()
        else:
            response.application_status = rapp_manager_msgs.Constants.APP_STOPPED
            response.application = rapp_manager_msgs.App()
        if self._remote_name:
            response.remote_controller = self._remote_name
        else:
            response.remote_controller = rapp_manager_msgs.Constants.NO_REMOTE_CONNECTION
        response.application_namespace = self._application_namespace
        return response

    def _get_app_msg_list(self, app_list_dictionary):
        app_msg_list = []
        for app in app_list_dictionary.values():
            app_msg_list.append(app.to_msg())
        return app_msg_list

    def _process_get_runnable_app_list(self, req):
        response = rapp_manager_srvs.GetAppListResponse()
        response.available_apps.extend(self._get_app_msg_list(self._runnable_apps))
        response.running_apps = []
        if self._current_rapp:
            response.running_apps.append(self._current_rapp.to_msg())
        return response

    def _publish_app_list(self):
        '''
          Publishes an updated list of available and running apps (in that order).
        '''
        app_list = rapp_manager_msgs.AppList()
        try:
            if self._current_rapp:
                app_list.available_apps.extend(self._get_app_msg_list(self._runnable_apps))
                app_list.running_apps = [self._current_rapp.to_msg()]
            else:
                app_list.available_apps.extend(self._get_app_msg_list(self._runnable_apps))
                app_list.running_apps = []
            self._publishers['app_list'].publish(app_list)
        except KeyError:
            pass
        except rospy.exceptions.ROSException:  # publishing to a closed topic.
            pass

    def _process_start_app(self, req):
        resp = rapp_manager_srvs.StartAppResponse()
        resp.app_namespace = self._application_namespace
        rospy.loginfo("App Manager : request received to start app [%s]" % req.name)

        # check is the app is already running
        if self._current_rapp:
            resp.started = False
            resp.message = "an app is already running [%s]" % self._current_rapp.data['name']
            rospy.logwarn("App Manager : %s" % resp.message)
            return resp

        # check if the app can be run
        try:
            rapp = self._runnable_apps[req.name]
        except KeyError:
            resp.started = False

            # check if app is installed
            #    Since we use indexer from now, it is hard to know whether rapp is installed or not. 
            #    This will return once we see a necesity of informative message
            #if not req.name in self._preinstalled_apps:
            #    resp.message = ("The requested app '%s' is not installed." % req.name)
            #    rospy.logwarn("App Manager : %s" % resp.message)
            #    return resp
            resp.message = ("The requested app '%s' is installed, but cannot be started"
                            ", because its required capabilities are not available." % req.name)
            rospy.logwarn("App Manager : %s" % resp.message)
            return resp

        # check if the app requires capabilities
        caps_list = self.caps_list if 'required_capabilities' in self._runnable_apps[req.name].data else None

        if caps_list:
            rospy.loginfo("App Manager : starting required capabilities.")
            result, message = start_capabilities_for_rapp(self._runnable_apps[req.name].data['required_capabilities'], self.caps_list)

            if not result: # if not none, it failed to start capabilities
                resp.started = False
                resp.message = message  
                return resp

        rospy.loginfo("App Manager : starting app '" + req.name + "' underneath " + self._application_namespace)
        resp.started, resp.message, subscribers, publishers, services, action_clients, action_servers = \
                    rapp.start(self._application_namespace,
                               self._gateway_name,
                               self._rocon_uri,
                               req.remappings,
                               self._param['app_output_to_screen'],
                               self.caps_list)

        rospy.loginfo("App Manager : %s" % self._remote_name)
        # small pause (convenience only) to let connections to come up
        # gateway watcher usually rolls over slowly. so this makes sure the flips get enacted on promptly
        rospy.rostime.wallsleep(0.5)
        if self._remote_name:
            self._flip_connections(self._remote_name, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER)
            self._flip_connections(self._remote_name, publishers, gateway_msgs.ConnectionType.PUBLISHER)
            self._flip_connections(self._remote_name, services, gateway_msgs.ConnectionType.SERVICE)
            self._flip_connections(self._remote_name, action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT)
            self._flip_connections(self._remote_name, action_servers, gateway_msgs.ConnectionType.ACTION_SERVER)
        if resp.started:
            self._current_rapp = rapp
            self._publish_app_list()
            thread.start_new_thread(self._monitor_rapp, ())
        return resp

    def _process_stop_app(self, req=None):
        '''
          Stops a currently running rapp. This can be triggered via the stop_app service call (in which
          case req is configured), or if the rapp monitoring thread detects that it has
          naturally stopped by itself (in which case req is None).

          :param req: variable configured when triggered from the service call.
        '''
        resp = rapp_manager_srvs.StopAppResponse()
        if not self._current_rapp:
            resp.stopped = False
            resp.error_code = rapp_manager_msgs.ErrorCodes.RAPP_IS_NOT_RUNNING
            resp.message = "tried to stop a rapp, but no rapp found running"
            rospy.logwarn("App Manager : Received a request to stop a rapp, but no rapp found running.")
            return resp

        rapp_name = self._current_rapp.data['name']
        rospy.loginfo("App Manager : Stopping rapp '" + rapp_name + "'.")

        resp.stopped, resp.message, subscribers, publishers, services, action_clients, action_servers = \
                self._current_rapp.stop()

        if self._remote_name:
            self._flip_connections(self._remote_name, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER, cancel_flag=True)
            self._flip_connections(self._remote_name, publishers, gateway_msgs.ConnectionType.PUBLISHER, cancel_flag=True)
            self._flip_connections(self._remote_name, services, gateway_msgs.ConnectionType.SERVICE, cancel_flag=True)
            self._flip_connections(self._remote_name, action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT, cancel_flag=True)
            self._flip_connections(self._remote_name, action_servers, gateway_msgs.ConnectionType.ACTION_SERVER, cancel_flag=True)

        if resp.stopped:
            self._current_rapp = None
            self._publish_app_list()
            if 'required_capabilities' in self._runnable_apps[rapp_name].data:
                rospy.loginfo("App Manager : Stopping required capabilities.")
                result, message = start_capabilities_for_rapp(self._runnable_apps[rapp_name].data['required_capabilities'], self.caps_list)
                if not result: # if not none, it failed to start capabilities
                    resp.stop= False
                    resp.message = message  
                    return resp
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

    def _advertise_services(self, service_names):
        '''
          Advertise rocon_app_manager services via the gateway,
          if it is available.

          :param service_names: rocon app manager service names
          :type service_names: strs
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

          :param remote_name: the name of the remote gateway to flip to.
          :type remote_name: str
          :param connection_names: the topic/service/action_xxx names
          :type connetion_names: list of str
          :param connection_type: publisher, subscriber, service, action_client or action_server
          :type connection_type: gateway_msgs.ConnectionType
          :param cancel_flag: whether or not we are flipping (false) or unflipping (true)
          :type cancel_flag: bool
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
            if resp.result == gateway_msgs.ErrorCodes.NO_HUB_CONNECTION and cancel_flag:
                # can often happen if a concert goes down and brings the hub down as as well
                # so just suppress this warning if it's a request to cancel
                rospy.logwarn("App Manager : failed to cancel flips (probably remote hub intentionally went down as well) [%s, %s]" % (resp.result, resp.error_message))
            else:
                rospy.logerr("App Manager : failed to flip [%s, %s]" % (resp.result, resp.error_message))

    def spin(self):
        while not rospy.is_shutdown():
            gateway_info = self._gateway_services['gateway_info'](timeout=rospy.Duration(0.3))
            if gateway_info:
                if gateway_info.connected:
                    self._gateway_name = gateway_info.name
                    self._gateway_ip = gateway_info.ip
                    if self._init_services():
                        break
            # don't need a sleep since our timeout on the service call acts like this.
        rospy.spin()
