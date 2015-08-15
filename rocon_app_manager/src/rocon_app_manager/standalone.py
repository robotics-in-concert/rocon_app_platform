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
import rocon_python_comms
import rocon_python_utils
import rocon_app_utilities
import rocon_app_utilities.rapp_repositories as rapp_repositories

# local imports
from . import exceptions
from ros_parameters import setup_ros_parameters
from .rapp import convert_rapps_from_rapp_specs
from .utils import plurality_converter

##############################################################################
# Rapp Manager
##############################################################################


class Services:
    def __init__(self, services):
        """
        Converts the incoming list of service name, service type, callback function triples into proper variables of this class.

        :param services: incoming list of service specifications
        :type services: list of (str, str, function) tuples representing (service_name, service_type, callback) pairs.
        """
        self.__dict__ = {rocon_python_comms.basename(service_name): rospy.Service(service_name, service_type, callback) for (service_name, service_type, callback) in services}


class Publishers:
    def __init__(self, publishers):
        """
        Converts the incoming list of publisher name, type, latched, queue_size specifications into proper variables of this class.

        :param publishers: incoming list of service specifications
        :type publishers: list of (str, str, bool, int) tuples representing (publisher_name, publisher_type, latched, queue_size) specifications.
        """
        self.__dict__ = {rocon_python_comms.basename(publisher_name): rospy.Publisher(publisher_name, publisher_type, latched, queue_size) for (publisher_name, publisher_type, latched, queue_size) in publishers}


# class Subscribers:
#     def __init__(self, subscribers):
#         """
#         Converts the incoming list of publisher name, service type pairs into proper variables of this class.
# 
#         :param subscribers: incoming list of service specifications
#         :type subscribers: list of (str, str, bool, int) tuples representing (publisher_name, publisher_type, latched, queue_size) specifications.
#         """
#         self.__dict__ = {rocon_python_comms.basename(subscriber_name): rospy.Publisher(subscriber_name, subscriber_type, latched, queue_size) for (subscriber_name, publisher_type, latched, queue_size) in publishers}


class Standalone(object):
    '''
        Standalone version of the rapp manager. An instance of this type
        only concerns itself with one job - that of rapp management.

        :ivar current_rapp: currently running rapp, otherwise None
        :vartype current_rapp: rocon_app_manager.Rapp
        :ivar application_namespace: launch rapps under this namespace, defaults to this node's private namespace
        :vartype application_namespace: str
        :ivar rocon_uri: this platform's signature used to determine if a rapp is compatible or not
        :vartype rocon_uri: str
        :ivar services: all ros services this instance provides
        :vartype: rocon_python_comms.Services
        :ivar publishers: all ros publishers this instance provides
        :vartype: rocon_python_comms.Publishers
        :ivar indexer: indexes all rapps on the system
        :vartype indexer: rocon_app_utilities.RappIndexer
        :ivar _parameters: all ros parameters for this instance organised in a dictionary
        :vartype _parameters: dict of string:value pairs.
    '''

    ##########################################################################
    # Initialisation
    ##########################################################################
    def __init__(self):
        self.current_rapp = None
        self.application_namespace = None  # Push all app connections underneath this namespace
        roslaunch.pmon._init_signal_handlers()
        self.parameters = setup_ros_parameters()
        self.rocon_uri = self._generate_rocon_uri()

        self.services = Services(
            [
                ('platform_info', rocon_std_srvs.GetPlatformInfo, self._process_platform_info),
                ('list_rapps', rapp_manager_srvs.GetRappList, self._process_get_runnable_rapp_list),
                ('start_rapp', rapp_manager_srvs.StartRapp, self._process_start_app),
                ('stop_rapp', rapp_manager_srvs.StopRapp, self._process_stop_app)
            ]
        )
        latched = True
        queue_size_five = 5
        self.publishers = Publishers(
            [
                ('status', rapp_manager_msgs.Status, latched, queue_size_five),
                ('rapp_list', rapp_manager_msgs.RappList, latched, queue_size_five),
                ('incompatible_rapp_list', rapp_manager_msgs.IncompatibleRappList, latched, queue_size_five)
            ]
        )
        self.caps_list = {}

        rospy.loginfo("Rapp Manager : indexing rapps...")
        self.indexer = rapp_repositories.get_combined_index(package_whitelist=self.parameters['rapp_package_whitelist'], package_blacklist=self.parameters['rapp_package_blacklist'])

        if self.parameters['auto_rapp_installation']:
            try:
                self._dependency_checker = rocon_app_utilities.DependencyChecker()
                rospy.loginfo("Rapp Manager : auto rapp installation is enabled ..")
            except KeyError as unused_e:
                rospy.logwarn("Rapp Manager : fails to initialise auto rapp installer. Disabling auto_rapp_installation ..")
                self.parameters['auto_rapp_installation'] = False
        self._runnable_apps, self._installable_apps, self._noninstallable_rapps, self._platform_filtered_apps, self._capabilities_filtered_apps, self._invalid_apps = self._determine_runnable_rapps()

        self._preferred = {}
        self._configure_preferred_rapp_for_virtuals()

        self._init_default_service_names()
        self._init_gateway_services()
        #if we dont depend on gateway we can init services right now
        if not self.parameters['use_gateway_uuids']:
            self._init_services()

        self._debug_ignores = {}  # a remote_controller_name : timestamp of the last time we logged an ignore response

        rospy.loginfo("Rapp Manager : initialised.")

    def _generate_rocon_uri(self):
        '''
          Initialises the rapp manager with the appropriate platform information
          represented as a rocon_uri.
        '''
        # This might be naive and only work well on ubuntu...
        os_codename = rospkg.os_detect.OsDetect().get_codename()
        rocon_uri = "rocon:/" + self.parameters['robot_type'].lower().replace(' ', '_') + \
                    "/" + self.parameters['robot_name'].lower().replace(' ', '_') + \
                    "/" + rocon_python_utils.ros.get_rosdistro() + \
                    "/" + os_codename
        return rocon_uri


    def _determine_runnable_rapps(self):
        '''
         Prune unsupported apps due to lack of support for required capabilities.

         :returns: incompatible app list dictionaries for capability incompatibilities respectively
         :rtype: {rocon_app_manager.Rapp}, [str], [str]
        '''
        rospy.loginfo("Rapp Manager : determining runnable rapps...")
        compatible_rapps, platform_incompatible_rapps, invalid_rapps = .get_compatible_rapps(uri=self._rocon_uri, ancestor_share_check=False)
        runnable_rapp_specs, capabilities_incompatible_rapps = self._filter_capability_unavailable_rapps(compatible_rapps)

        if self.parameters['auto_rapp_installation']:
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
        full_apps = {}
        full_apps.update(self._runnable_apps)
        full_apps.update(self._installable_apps)

        for name, rapp in full_apps.items():
            ancestor_name = rapp.data['ancestor_name']
            if not ancestor_name in virtual_apps.keys():
                virtual_apps[ancestor_name] = []
                virtual_apps[ancestor_name].append(rapp)
            else:
                virtual_apps[ancestor_name].append(rapp)

        # Get preferred rapp configurations from parameter server and use
        preferred = {}
        v_rapps = {}
        for pair in self.parameters['preferred']:
            preferred.update(pair)

        for rapp_name, available_rapps in virtual_apps.items():
            if not rapp_name in preferred:
                if len(available_rapps) > 1:
                    v_rapps[rapp_name] = available_rapps[0]
                    rospy.logwarn("Rapp Manager : '" + rapp_name + "' is not unique and has no preferred rapp. " + available_rapps[0].data['name'] + "' has been selected.")
                else:
                    v_rapps[rapp_name] = available_rapps[0]
                continue
            else:
                preferred_rapp_name = preferred[rapp_name]

                if preferred_rapp_name not in full_apps:
                    rospy.logwarn("Rapp Manager : Given preferred rapp '" + preferred_rapp_name + "' for '" + rapp_name + "' does not exist. '" + available_rapps[0].data['name'] + "' has been selected.")
                    v_rapps[rapp_name] = available_rapps[0]
                else:
                    rospy.loginfo("Rapp Manager: '%s' -> '%s'" % (rapp_name, preferred_rapp_name))
                    v_rapps[rapp_name] = full_apps[preferred_rapp_name]
        self._virtual_apps = v_rapps
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
        if self.parameters['local_remote_controllers_only'] and not req.cancel:
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
        elif req.remote_target_name in self.parameters['remote_controller_whitelist']:
            response = self._accept_invitation(req)
        elif len(self.parameters['remote_controller_whitelist']) == 0 and req.remote_target_name not in self.parameters['remote_controller_blacklist']:
            response = self._accept_invitation(req)
        else:
            # If we get here, we are not permitted to accept, either not in the whitelist, or in the blacklist
            if len(self.parameters['remote_controller_whitelist']) != 0:
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
            self._application_namespace = self._gateway_name.lower().replace(' ', '_') if self._gateway_name else self.parameters['robot_name']
        else:
            self._application_namespace = req.application_namespace

        # Flips/Unflips
        try:
            connections = {'services': [self._service_names['start_rapp'], self._service_names['stop_rapp']]}
            self._flip_all_connections(req.remote_target_name,
                                   connections,
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
        published_interfaces = []
        published_parameters = []
        rapp = rapp_manager_msgs.Rapp()
        rapp_status = rapp_manager_msgs.Status.RAPP_STOPPED
        if self._current_rapp:
            try:
                published_interfaces = self._current_rapp.published_interfaces_to_msg_list()
                published_parameters = self._current_rapp.published_parameters_to_msg_list()
                rapp = self._current_rapp.to_msg()
                rapp_status = rapp_manager_msgs.Status.RAPP_RUNNING
            except AttributeError:  # i.e. current_rapp is None
                # catch when self._current_rapp is NoneType since there is a miniscule chance
                # it might have changed inbetween the if check and the method calls
                pass  # nothing to do here as we predefine everything for this case.
        if self._remote_name:
            remote_controller = self._remote_name
        else:
            remote_controller = rapp_manager_msgs.Constants.NO_REMOTE_CONTROLLER
        msg = rapp_manager_msgs.Status(application_namespace=self._application_namespace,
                                       remote_controller=remote_controller,
                                       rapp_status=rapp_status,
                                       rapp=rapp,
                                       published_interfaces=published_interfaces,
                                       published_parameters=published_parameters
                                       )
        try:
            self._publishers['status'].publish(msg)
        except rospy.ROSException:  # publisher has unregistered - ros shutting down
            pass

    def _publish_rapp_list(self):
        '''
          Publishes an updated list of available apps (in that order).
        '''
        rapp_list = rapp_manager_msgs.RappList()
        try:
            rapp_list.available_rapps = self._get_available_rapp_list()
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

        rospy.loginfo(
            "Rapp Manager : starting app '" + req.name +
            (("' underneath '" + self._application_namespace + "'") if self._application_namespace else "'")
        )
        resp.started, resp.message, connections = rapp.start(self._application_namespace,
                                                           self._gateway_name,
                                                           self._rocon_uri,
                                                           req.remappings,
                                                           req.parameters,
                                                           self.parameters['app_output_to_screen'],
                                                           self.parameters['simulation'],
                                                           self.caps_list)

        rospy.loginfo("Rapp Manager : Remote Name [%s]" % self._remote_name)

        # small pause (convenience only) to let connections to come up
        # gateway watcher usually rolls over slowly. so this makes sure the flips get enacted on promptly
        rospy.rostime.wallsleep(0.5)

        if resp.started:
            self._current_rapp = rapp
            self._publish_rapp_list()
            self._publish_status()
            thread.start_new_thread(self._monitor_rapp, ())

            if self._remote_name:
                success, message = self._flip_all_connections(self._remote_name, connections, False)
                if not success:
                    self._process_stop_app()
                    resp.started = success
                    resp.message = message
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

        resp.stopped, resp.message, connections = rapp.stop()

        if self._remote_name:
            success = self._flip_all_connections(self._remote_name, connections, cancel_flag=True)

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

    def _flip_all_connections(self, remote_name, connections, cancel_flag=False):
        '''
          (Un)Flip connections to a remote gateway.

          :param remote_name: the name of the remote gateway to flip to.
          :type remote_name: str
          :param connections: the dict of connection types(topic/service/action_xxx) names
          :type connetions: dict
          :param cancel_flag: whether or not we are flipping (false) or unflipping (true)
          :type cancel_flag: bool

          :returns: success or not, message 
          :rtype: bool, str
        '''
        success = False
        message = ""

        req = gateway_srvs.RemoteRequest()
        req.cancel = cancel_flag
        req.remotes = []

        # Create request
        for conn_type, conns in connections.items():
            gateway_type = plurality_converter[conn_type]
            for conn_name in conns:
                remote_rule = create_gateway_remote_rule(remote_name, create_gateway_rule(conn_name, gateway_type))
                req.remotes.append(remote_rule)

        # Send request
        attempts = 0
        MAX_ATTEMPTS = 4
        while not success and attempts < MAX_ATTEMPTS and not rospy.is_shutdown():
            try:
                resp = self._gateway_services['flip'](req)
            except rospy.service.ServiceException:
                # often disappears when the gateway shuts down just before the app manager, ignore silently.
                success = False
                message = "Flip service has disappeared"
                attempts = attempts + 1
                continue

            if resp.result == gateway_msgs.ErrorCodes.SUCCESS:
                rospy.loginfo("Rapp Manager : successfully flipped %s" % str([(names) for names in connections.values()]))
                success = True
            else:
                if resp.result == gateway_msgs.ErrorCodes.NO_HUB_CONNECTION and cancel_flag:
                    # can often happen if a concert goes down and brings the hub down as as well
                    # so just suppress this warning if it's a request to cancel
                    rospy.logwarn("Rapp Manager : failed to cancel flips (probably remote hub intentionally went down as well) [%s, %s]" % (resp.result, resp.error_message))
                else:
                    message = "failed to flip [%s][%s, %s]" % (str(connections), resp.result, resp.error_message)
                    rospy.logerr("Rapp Manager : %s"%message)
                success = False
                attempts = attempts + 1
                rospy.sleep(1.0)
        return success, message

    def _check_runnable(self, requested_rapp_name):
        success = False
        message = ""
        rapp = None

        if requested_rapp_name in self._virtual_apps.keys():  # Virtual rapp
            rapp = self._virtual_apps[requested_rapp_name]
            success = True
            message = ""

            if rapp.data['name'] in self._installable_apps.keys():
                success, message, rapp = self._install_rapp(rapp.data['name'])

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

        if self.parameters['auto_rapp_installation']:
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
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            gateway_info = self._gateway_services['gateway_info'](timeout=rospy.Duration(0.3))
            if gateway_info:
                self._gateway_name = gateway_info.name
                self._gateway_ip = gateway_info.ip
                #if we depend on gateway we can init services now (only one time)
                if 0 == len(self._services) and 0 == len(self._publishers) and self.parameters['use_gateway_uuids']:
                    if self._init_services():
                        break
            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                rospy.loginfo("Rapp Manager : breaking out of spin loop [most likley just ros shutting down]")
        rospy.spin()
