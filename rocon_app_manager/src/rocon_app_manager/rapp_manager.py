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
from .caps_list import CapsList, start_capabilities_from_caps_list, stop_capabilities_from_caps_list
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
import rospkg.os_detect
import rocon_python_utils
import rocon_app_utilities
import rocon_app_utilities.rapp_repositories as rapp_repositories

# local imports
from . import exceptions
from ros_parameters import setup_ros_parameters
from .rapp import convert_rapps_from_rapp_specs

##############################################################################
# Rapp Manager
##############################################################################


class RappManager(object):
    '''
        Robot App Manager ~ Rocon App Manager
    '''

    ##########################################################################
    # Initialisation
    ##########################################################################
    def __init__(self):
        self._gateway_name = None  # Name of our local gateway (if available)
        self._gateway_ip = None  # IP/Hostname of our local gateway if available
        self._remote_name = None  # Name (gateway name) for the entity that is remote controlling this app manager
        self._current_rapp = None  # Rapp that is running, otherwise None
        self._application_namespace = None  # Push all app connections underneath this namespace
        roslaunch.pmon._init_signal_handlers()
        self._services = {}
        self._publishers = {}
        self._param = setup_ros_parameters()
        (self._rocon_uri, self._icon) = self._set_platform_info()

        self.caps_list = {}
        self._initialising_services = False

        rospy.loginfo("Rapp Manager : indexing rapps...")
        self._indexer = rapp_repositories.get_combined_index(package_whitelist=self._param['rapp_package_whitelist'], package_blacklist=self._param['rapp_package_blacklist'])

        if self._param['auto_rapp_installation']:
            try:
                self._dependency_checker = rocon_app_utilities.DependencyChecker(self._indexer)
                rospy.loginfo("Rapp Manager : auto rapp installation is enabled ..")
            except KeyError as e:
                rospy.logwarn("Rapp Manager : fails to initialise auto rapp installer. Disabling auto_rapp_installation ..")
                self._param['auto_rapp_installation'] = False
        self._runnable_apps, self._installable_apps, self._noninstallable_rapps, self._platform_filtered_apps, self._capabilities_filtered_apps, self._invalid_apps = self._determine_runnable_rapps()

        self._preferred = {}
        self._configure_preferred_rapp_for_virtuals()

        self._init_default_service_names()
        self._init_gateway_services()
        self._init_services()

        if self._param['auto_start_rapp']:  # None and '' are both false here
            request = rapp_manager_srvs.StartRappRequest(self._param['auto_start_rapp'], [])
            unused_response = self._process_start_app(request)

        self._debug_ignores = {}  # a remote_controller_name : timestamp of the last time we logged an ignore response

        rospy.loginfo("Rapp Manager : initialised.")

    def _set_platform_info(self):
        '''
          Initialises the rapp manager with the appropriate platform info.
          This is part of the __init__ process.
        '''
        # This might be naive and only work well on ubuntu...
        os_codename = rospkg.os_detect.OsDetect().get_codename()
        rocon_uri = "rocon:/" + self._param['robot_type'].lower().replace(' ', '_') + \
                    "/" + self._param['robot_name'].lower().replace(' ', '_') + \
                    "/" + rocon_python_utils.ros.get_rosdistro() + \
                    "/" + os_codename
        try:
            filename = rocon_python_utils.ros.find_resource_from_string(self._param['robot_icon'])
            icon = rocon_python_utils.ros.icon_to_msg(filename)
        except exceptions.NotFoundException:
            rospy.logwarn("Rapp Manager : icon resource not found [%s]" % self._param['robot_icon'])
            icon = rocon_std_msgs.Icon()
        except ValueError:
            rospy.logwarn("Rapp Manager : invalid resource name [%s]" % self._param['robot_icon'])
            icon = rocon_std_msgs.Icon()
        return (rocon_uri, icon)

    def _init_default_service_names(self):
        self._default_service_names = {}
        self._default_service_names['platform_info'] = 'platform_info'
        self._default_service_names['list_rapps'] = 'list_rapps'
        self._default_service_names['invite'] = 'invite'
        self._default_service_names['start_rapp'] = 'start_rapp'
        self._default_service_names['stop_rapp'] = 'stop_rapp'
        # Latched publishers
        self._default_publisher_names = {}
        self._default_publisher_names['status'] = 'status'
        self._default_publisher_names['rapp_list'] = 'rapp_list'
        self._default_publisher_names['incompatible_rapp_list'] = 'incompatible_rapp_list'

    def _init_gateway_services(self):
        self._gateway_services = {}
        self._gateway_services['gateway_info'] = rocon_python_comms.SubscriberProxy('~gateway_info', gateway_msgs.GatewayInfo)
        self._gateway_services['remote_gateway_info'] = rospy.ServiceProxy('~remote_gateway_info', gateway_srvs.RemoteGatewayInfo)
        self._gateway_services['flip'] = rospy.ServiceProxy('~flip', gateway_srvs.Remote)
        self._gateway_services['advertise'] = rospy.ServiceProxy('~advertise', gateway_srvs.Advertise)
        self._gateway_services['pull'] = rospy.ServiceProxy('~pull', gateway_srvs.Remote)
        self._gateway_publishers = {}
        self._gateway_publishers['force_update'] = rospy.Publisher("~force_update",
                                                                   std_msgs.Empty,
                                                                   queue_size=5
                                                                   )

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
        base_name = self._gateway_name.lower().replace(' ', '_') if self._gateway_name else ""  # latter option is for standalone mode
        for name in self._default_service_names:
            if (base_name == ""):
                self._service_names[name] = '~' + self._default_service_names[name]
            else:
                self._service_names[name] = '/' + base_name + '/' + self._default_service_names[name]
        for name in self._default_publisher_names:
            if(base_name == ""):
                self._publisher_names[name] = '~' + self._default_publisher_names[name]
                self._application_namespace = ""
            else:
                self._publisher_names[name] = '/' + base_name + '/' + self._default_publisher_names[name]
                self._application_namespace = base_name
        try:
            # Advertisable services - we advertise these by default advertisement rules for the app manager's gateway.
            self._services['platform_info'] = rospy.Service(self._service_names['platform_info'], rocon_std_srvs.GetPlatformInfo, self._process_platform_info)
            self._services['list_rapps'] = rospy.Service(self._service_names['list_rapps'], rapp_manager_srvs.GetRappList, self._process_get_runnable_rapp_list)
            self._services['invite'] = rospy.Service(self._service_names['invite'], rapp_manager_srvs.Invite, self._process_invite)
            # Flippable services
            self._services['start_rapp'] = rospy.Service(self._service_names['start_rapp'], rapp_manager_srvs.StartRapp, self._process_start_app)
            self._services['stop_rapp'] = rospy.Service(self._service_names['stop_rapp'], rapp_manager_srvs.StopRapp, self._process_stop_app)
            # Latched publishers
            self._publishers['status'] = rospy.Publisher(self._publisher_names['status'], rapp_manager_msgs.Status, latch=True, queue_size=5)
            self._publishers['rapp_list'] = rospy.Publisher(self._publisher_names['rapp_list'], rapp_manager_msgs.RappList, latch=True, queue_size=1)
            self._publishers['incompatible_rapp_list'] = rospy.Publisher(self._publisher_names['incompatible_rapp_list'], rapp_manager_msgs.IncompatibleRappList, latch=True, queue_size=1)
            # Force an update on the gateway
            self._gateway_publishers['force_update'].publish(std_msgs.Empty())
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            self._initialising_services = False
            return False
        # initialise the latched publishers
        self._publish_status()
        self._publish_rapp_list()
        # TODO: Currently blacklisted apps and non-whitelisted apps are not provided yet
        self._publishers['incompatible_rapp_list'].publish([], [], self._platform_filtered_apps, self._capabilities_filtered_apps)
        self._initialising_services = False
        return True

    def _determine_runnable_rapps(self):
        '''
         Prune unsupported apps due to lack of support for required capabilities.

         :returns: incompatible app list dictionaries for capability incompatibilities respectively
         :rtype: {rocon_app_manager.Rapp}, [str], [str]
        '''
        rospy.loginfo("Rapp Manager : determining runnable rapps...")
        compatible_rapps, platform_incompatible_rapps, invalid_rapps = self._indexer.get_compatible_rapps(uri=self._rocon_uri, ancestor_share_check=False)
        runnable_rapp_specs, capabilities_incompatible_rapps = self._filter_capability_unavailable_rapps(compatible_rapps)

        if self._param['auto_rapp_installation']:
            runnable_rapp_specs, installable_rapp_specs, noninstallable_rapp_specs = self._determine_installed_rapps(runnable_rapp_specs)
            installable_rapps = convert_rapps_from_rapp_specs(installable_rapp_specs)
        else:
            installable_rapps = {}
            noninstallable_rapp_specs = {}
        runnable_rapps = convert_rapps_from_rapp_specs(runnable_rapp_specs)

        # Log out the rapps
        for rapp_name, reason in invalid_rapps.items():
            rospy.logwarn("Rapp Manager : '" + rapp_name + "' is invalid [" + str(reason) + "]")

        for rapp in platform_incompatible_rapps.values():
            rospy.logwarn("Rapp Manager : '" + str(rapp.resource_name) + "' is incompatible [" + rapp.raw_data['compatibility'] + "][" + self._rocon_uri + "]")

        for rapp_name, reason in capabilities_incompatible_rapps.items():
            rospy.logwarn("Rapp Manager : '" + rapp_name + "' is compatible, but is missing capabilities [" + str(reason) + "]")

        for rapp_name, reason in noninstallable_rapp_specs.items():
            rospy.logwarn("Rapp Manager : '" + rapp_name + "' is compatible, but cannot be installed.")

        for rapp_name, unused_v in installable_rapps.items():
            rospy.loginfo("Rapp Manager : '" + rapp_name + "' added to the list of installable apps.")

        for rapp_name, unused_v in runnable_rapps.items():
            rospy.loginfo("Rapp Manager : '" + rapp_name + "' added to the list of runnable apps.")

        noninstallable_rapps = noninstallable_rapp_specs.keys()
        platform_filtered_rapps = platform_incompatible_rapps.keys()
        capabilities_filtered_rapps = capabilities_incompatible_rapps.keys()

        return (runnable_rapps, installable_rapps, noninstallable_rapps, platform_filtered_rapps, capabilities_filtered_rapps, invalid_rapps)

    def _configure_preferred_rapp_for_virtuals(self):
        virtual_apps = {}
        #default_apps = setup_default_app_from_params()
        full_apps = {}
        full_apps.update(self._runnable_apps)
        full_apps.update(self._installable_apps)

        for name, rapp in full_apps.items():
            ancestor_name = rapp.data['ancestor_name']
            if not ancestor_name in virtual_apps.keys():
                virtual_apps[ancestor_name] = rapp

        # Get preferred rapp configurations from parameter server and use
        preferred = {}
        for pair in self._param['preferred']:
            preferred.update(pair)

        for rapp_name, selected_rapp in virtual_apps.items():
            selected_rapp_name = selected_rapp.data['name']
            if not rapp_name in preferred:
                rospy.logwarn("Rapp Manager : No preferred rapp for '" + rapp_name + "'.  '" + selected_rapp_name + "' has been selected.")
                continue
            preferred_rapp_name = preferred[rapp_name]
            if not preferred_rapp_name in full_apps:
                rospy.logwarn("Rapp Manager : Given preferred rapp '" + preferred_rapp_name + "' for '" + rapp_name + "' does not exist. '" + selected_rapp_name + "' has been selected.")
                continue
            rospy.loginfo("Rapp Manager: '%s' -> '%s'"%(rapp_name, preferred_rapp_name))
            virtual_apps[rapp_name] = full_apps[preferred_rapp_name]
        self._virtual_apps = virtual_apps
        self._preferred = preferred

    def _filter_capability_unavailable_rapps(self, compatible_rapps):
        '''
          Filters out rapps which does not meet the platform's capability

          :params compatible_rapps: Platform compatible rapp dict
          :type compatible_rapplist: dict

          :returns: runnable rapp, capability filtered rapp
          :rtype: dict, dict
        '''
        is_caps_available = self._init_capabilities()  # First try initialise the list of available capabilities
        capabilities_filtered_apps = {}
        runnable_apps = {}

        # Then add runnable apps to list
        for rapp_name, rapp in compatible_rapps.items():
            if not is_caps_available:
                if 'required_capabilities' in rapp.data:
                    reason = "cannot be run, since capabilities are not available. Rapp will be excluded from the list of runnable apps."
                    capabilities_filtered_apps[rapp_name] = reason
                else:
                    runnable_apps[rapp_name] = rapp
            else:
                try:
                    self.caps_list.compatibility_check(rapp)
                    runnable_apps[rapp_name] = rapp
                except exceptions.MissingCapabilitiesException as e:
                    reason = "cannot be run, since some required capabilities (" + str(e.missing_caps) + ") are not installed. Rapp will be excluded from the list of runnable rapps."
                    capabilities_filtered_apps[rapp_name] = reason
        return runnable_apps, capabilities_filtered_apps

    def _determine_installed_rapps(self, rapps):
        '''
         Determines, which rapps have all their dependencies installed and which not.

          :params rapps: a list of rapps
          :type rapps: dict

          :returns: runnable rapps, installable rapps, noninstallable rapps
          :rtype: dict, dict, dict
        '''
        rospy.loginfo("Rapp Manager : determining installed rapps...")

        rapp_names = []
        for rapp in rapps:
            rapp_names.append(rapp)
        rapp_deps = self._dependency_checker.check_rapp_dependencies(rapp_names)

        runnable_rapps = {}
        installable_rapps = {}
        noninstallable_rapps = {}
        for rapp in rapp_deps:
            if rapp_deps[rapp].all_installed():
                runnable_rapps[rapp] = rapps[rapp]
            elif rapp_deps[rapp].any_not_installable():
                noninstallable_rapps[rapp] = rapps[rapp]
            else:
                installable_rapps[rapp] = rapps[rapp]

        return (runnable_rapps, installable_rapps, noninstallable_rapps)

    def _init_capabilities(self):
        try:
            self.caps_list = CapsList()
        except exceptions.NotFoundException as e:
            if 'Timed out' in str(e) or "Couldn't find" in str(e):
                rospy.loginfo("Rapp Manager : disabling apps requiring capabilities [%s]" % str(e))
            else:
                rospy.logwarn("Rapp Manager : disabling apps requiring capabilities [%s]" % str(e))
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
            self._publish_status()
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
            rospy.logwarn("Rapp Manager : ignoring request from %s to cancel invitation as it is not the current remote controller [%s]" % (str(req.remote_target_name), self._remote_name))
            return rapp_manager_srvs.InviteResponse(False, rapp_manager_msgs.ErrorCodes.NOT_CURRENT_REMOTE_CONTROLLER, "ignoring request from %s to cancel invitation as it is not the current remote controller [%s]")
        if not req.cancel and self._remote_name is not None:
            if req.remote_target_name == self._remote_name:
                rospy.logwarn("Rapp Manager : bastards are sending us repeat invites, so we ignore - we are already working for them! [%s]" % self._remote_name)
                return rapp_manager_srvs.InviteResponse(True, rapp_manager_msgs.ErrorCodes.SUCCESS, "you are a bastard, stop spamming us - you already invited this app manager!")
            else:
                if (req.remote_target_name not in self._debug_ignores or
                   ((rospy.Time.now() - self._debug_ignores[req.remote_target_name]) > rospy.Duration(120))):
                    self._debug_ignores[req.remote_target_name] = rospy.Time.now()
                    rospy.loginfo("Rapp Manager : ignoring invitation from %s [already invited by %s]" % (str(req.remote_target_name), self._remote_name))
                return rapp_manager_srvs.InviteResponse(False, rapp_manager_msgs.ErrorCodes.ALREADY_REMOTE_CONTROLLED, "already remote controlled from %s" % self._remote_name)
        # Variable setting
        if req.application_namespace == '':
            self._application_namespace = self._gateway_name.lower().replace(' ', '_') if self._gateway_name else self._param['robot_name']
        else:
            self._application_namespace = req.application_namespace

        # Flips/Unflips
        try:
            self._flip_connections(req.remote_target_name,
                                   [self._service_names['start_rapp'], self._service_names['stop_rapp']],
                                   gateway_msgs.ConnectionType.SERVICE,
                                   req.cancel
                                   )
        except Exception as unused_e:
            traceback.print_exc(file=sys.stdout)
            return False
        # Cleaning up and setting final state
        if req.cancel:
            if req.remote_target_name == self._remote_name:
                rospy.loginfo("Rapp Manager : cancelling remote control of this system [%s]" % str(req.remote_target_name))
                if self._current_rapp:
                    thread = threading.Thread(target=self._process_stop_app)
                    thread.start()
                self._remote_name = None
        else:
            rospy.loginfo("Rapp Manager : accepting invitation to be remote controlled [%s]" % str(req.remote_target_name))
            self._remote_name = req.remote_target_name
        return rapp_manager_srvs.InviteResponse(True, rapp_manager_msgs.ErrorCodes.SUCCESS, "success")

    def _process_platform_info(self, req):
        platform_info = rocon_std_msgs.PlatformInfo(version=rocon_std_msgs.Strings.ROCON_VERSION,
                                                    uri=self._rocon_uri,
                                                    icon=self._icon)
        return rocon_std_srvs.GetPlatformInfoResponse(platform_info)

    def _get_rapp_msg_list(self, app_list_dictionary):
        app_msg_list = []
        for app in app_list_dictionary.values():
            app_msg_list.append(app.to_msg())
        return app_msg_list

    def _get_available_rapp_list(self):
        avail = {}
        for name, rapp in self._virtual_apps.items():
            avail[name] = rapp.to_msg()
            avail[name].name = name

        for name, rapp in avail.items():
            if name in self._preferred:
                rapp.preferred = self._preferred[name]
            
        for name, rapp in self._runnable_apps.items():
            ancestor_name = rapp.data['ancestor_name']
            avail[ancestor_name].implementations.append(name)

        for name, rapp in self._installable_apps.items():
            ancestor_name = rapp.data['ancestor_name']
            avail[ancestor_name].implementations.append(name)

        return avail.values()

    def _process_get_runnable_rapp_list(self, req):
        response = rapp_manager_srvs.GetRappListResponse()
        response.available_rapps = self._get_available_rapp_list()
        response.running_rapps = []
        if self._current_rapp:
            response.running_rapps.append(self._current_rapp.to_msg())
        return response

    def _publish_status(self):
        """
         Publish status updates whenever something significant changes, e.g.
         remote controller changed, or rapp started/stopped.
        """
        if self._current_rapp:
            rapp = self._current_rapp.to_msg()
            rapp_status = rapp_manager_msgs.Status.RAPP_RUNNING
        else:
            rapp = rapp_manager_msgs.Rapp()
            rapp_status = rapp_manager_msgs.Status.RAPP_STOPPED
        if self._remote_name:
            remote_controller = self._remote_name
        else:
            remote_controller = rapp_manager_msgs.Constants.NO_REMOTE_CONTROLLER
        msg = rapp_manager_msgs.Status(application_namespace=self._application_namespace,
                                       remote_controller=remote_controller,
                                       rapp_status=rapp_status,
                                       rapp=rapp
                                       )
        try:
            self._publishers['status'].publish(msg)
        except rospy.ROSException:  # publisher has unregistered - ros shutting down
            pass

    def _publish_rapp_list(self):
        '''
          Publishes an updated list of available and running apps (in that order).
        '''
        rapp_list = rapp_manager_msgs.RappList()
        try:
            rapp_list.available_rapps = self._get_available_rapp_list()
            if self._current_rapp:
                rapp_list.running_rapps = [self._current_rapp.to_msg()]
            else:
                rapp_list.running_rapps = []
            self._publishers['rapp_list'].publish(rapp_list)
        except rospy.exceptions.ROSException:  # publishing to a closed topic.
            pass

    def _process_start_app(self, req):
        resp = rapp_manager_srvs.StartRappResponse()
        resp.application_namespace = self._application_namespace
        rospy.loginfo("Rapp Manager : request received to start rapp [%s]" % req.name)

        # check is the app is already running
        if self._current_rapp:
            resp.started = False
            resp.message = "an app is already running [%s]" % self._current_rapp.data['name']
            rospy.logwarn("Rapp Manager : %s" % resp.message)
            return resp

        # check if the app can be run
        success, reason, rapp = self._check_runnable(req.name)

        if not success:
            resp.started = False
            resp.message = reason
            rospy.logwarn("Rapp Manager : %s" % reason)
            return resp

        # check if the app requires capabilities
        caps_list = self.caps_list if 'required_capabilities' in rapp.data else None
        if caps_list:
            rospy.loginfo("Rapp Manager : Starting required capabilities.")
            result, message = start_capabilities_from_caps_list(rapp.data['required_capabilities'], self.caps_list)

            if not result:  # if not none, it failed to start capabilities
                resp.started = False
                resp.message = message
                return resp

        rospy.loginfo("Rapp Manager : starting app '" + req.name + "' underneath " + self._application_namespace)
        resp.started, resp.message, subscribers, publishers, services, action_clients, action_servers = \
            rapp.start(self._application_namespace,
                       self._gateway_name,
                       self._rocon_uri,
                       req.remappings,
                       req.parameters,
                       self._param['app_output_to_screen'],
                       self._param['simulation'],
                       self.caps_list)

        rospy.loginfo("Rapp Manager : %s" % self._remote_name)
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
            self._publish_rapp_list()
            self._publish_status()
            thread.start_new_thread(self._monitor_rapp, ())
        return resp

    def _process_stop_app(self, req=None):
        '''
          Stops a currently running rapp. This can be triggered via the stop_app service call (in which
          case req is configured), or if the rapp monitoring thread detects that it has
          naturally stopped by itself (in which case req is None).

          :param req: variable configured when triggered from the service call.
        '''
        resp = rapp_manager_srvs.StopRappResponse()
        try:  # use a try-except block here so the check is atomic
            rapp_name = self._current_rapp.data['name']
            self._current_rapp = None  # immediately prevent it from trying to stop the app again
        except AttributeError:
            resp.stopped = False
            resp.error_code = rapp_manager_msgs.ErrorCodes.RAPP_IS_NOT_RUNNING
            resp.message = "Tried to stop a rapp, but no rapp found running."
            rospy.logwarn("Rapp Manager : %s" % resp.message)
            return resp

        rospy.loginfo("Rapp Manager : Stopping rapp '" + rapp_name + "'.")

        success, reason, rapp = self._check_runnable(rapp_name)
        if not success:
            resp.started = False
            resp.error_code = rapp_manager_msgs.ErrorCodes.RAPP_IS_NOT_AVAILABLE
            resp.message = reason
            rospy.logwarn("Rapp Manager : %s" % resp.message)
            return resp

        resp.stopped, resp.message, subscribers, publishers, services, action_clients, action_servers = rapp.stop()

        if self._remote_name:
            self._flip_connections(self._remote_name, subscribers, gateway_msgs.ConnectionType.SUBSCRIBER, cancel_flag=True)
            self._flip_connections(self._remote_name, publishers, gateway_msgs.ConnectionType.PUBLISHER, cancel_flag=True)
            self._flip_connections(self._remote_name, services, gateway_msgs.ConnectionType.SERVICE, cancel_flag=True)
            self._flip_connections(self._remote_name, action_clients, gateway_msgs.ConnectionType.ACTION_CLIENT, cancel_flag=True)
            self._flip_connections(self._remote_name, action_servers, gateway_msgs.ConnectionType.ACTION_SERVER, cancel_flag=True)

        if resp.stopped:
            if 'required_capabilities' in rapp.data:
                rospy.loginfo("Rapp Manager : Stopping required capabilities.")
                result, message = stop_capabilities_from_caps_list(rapp.data['required_capabilities'], self.caps_list)
                if not result:  # if not none, it failed to stop capabilities
                    resp.stopped = False
                    resp.message = message
            self._publish_rapp_list()
            self._publish_status()
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
            rospy.loginfo("Rapp Manager : successfully flipped %s" % str([os.path.basename(name) for name in connection_names]))
        else:
            if resp.result == gateway_msgs.ErrorCodes.NO_HUB_CONNECTION and cancel_flag:
                # can often happen if a concert goes down and brings the hub down as as well
                # so just suppress this warning if it's a request to cancel
                rospy.logwarn("Rapp Manager : failed to cancel flips (probably remote hub intentionally went down as well) [%s, %s]" % (resp.result, resp.error_message))
            else:
                rospy.logerr("Rapp Manager : failed to flip [%s, %s]" % (resp.result, resp.error_message))

    def _check_runnable(self, requested_rapp_name):
        success = False
        message = ""
        rapp = None

        if requested_rapp_name in self._virtual_apps.keys():  # Virtual rapp
            rapp = self._virtual_apps[requested_rapp_name]
            success = True
            message = ""
        elif requested_rapp_name in self._runnable_apps.keys():  # Implementation Rapp
            rapp = self._runnable_apps[requested_rapp_name]
            success = True
            message = ""
        elif requested_rapp_name in self._installable_apps.keys():  # Installable Rapp
            success, message, rapp = self._install_rapp(requested_rapp_name)
        else:
            success = False
            message = ("The requested app '%s' is not among the runnable, nor installable rapps." % requested_rapp_name)

        return success, message, rapp

    def _install_rapp(self, requested_rapp_name):
        '''
        check if app can be installed
        '''
        success = False
        message = ""
        rapp = self._installable_apps[requested_rapp_name]

        if self._param['auto_rapp_installation']:
            rospy.loginfo("Rapp Manager : Installing rapp '" + rapp.data['name'] + "'")
            success, reason = rapp.install(self._dependency_checker)
            if success:
                rospy.loginfo("Rapp Manager : Rapp '" + rapp.data['name'] + "'has been installed.")
                # move rapp from installable to runnable
                self._runnable_apps[requested_rapp_name] = rapp
                del self._installable_apps[requested_rapp_name]
                # TODO : consider calling publish_rapp_list if we split publishing runnable and installed there.

                success = True
                message = ""
            else:
                success = False
                message = "Installing rapp '" + rapp.data['name'] + "' failed. Reason: " + str(reason)
        else:
            success = False
            url = "'http://wiki.ros.org/rocon_app_manager/Tutorials/indigo/Automatic Rapp Installation'"
            message = str("Rapp '" + rapp.data['name'] + "' can be installed, "
                          + "but automatic installation is not enabled. Please refer to " + str(url)
                          + " for instructions on how to set up automatic rapp installation.")
        return success, message, rapp

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
