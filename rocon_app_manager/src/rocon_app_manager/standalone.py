#
# License: BSD
#   https://raw.github.com/robotics-in-py/rocon_app_platform/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: standalone
   :platform: Unix
   :synopsis: A rapp manager class for standalone use.


Provides a class for instantiating a rapp manager for standalone use, i.e.
not multi-robot.

----

"""

##############################################################################
# Imports
##############################################################################

import rospy
import time
import thread
import roslaunch.pmon
from .caps_list import CapsList, start_capabilities_from_caps_list, stop_capabilities_from_caps_list
import rocon_python_comms
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_app_manager_msgs.srv as rapp_manager_srvs
import rocon_app_utilities
import rocon_app_utilities.rapp_repositories as rapp_repositories
import rocon_uri
import std_msgs.msg as std_msgs

# local imports
from . import exceptions
from . import utils
from ros_parameters import StandaloneParameters
from .rapp import convert_rapps_from_rapp_specs

##############################################################################
# Rapp Manager
##############################################################################


class Standalone(object):
    '''
        Standalone version of the rapp manager. An instance of this type
        only concerns itself with one job - that of rapp management.

        **Publishers**

        * **~status** (*rapp_manager_msgs.Status*) - running status of the rapp manager [latched]
        * **~rapp_list** (*rapp_manager_msgs.RappList*) - the successfully loaded rapp list [latched]
        * **~incompatible_rapp_list** (*rapp_manager_msgs.IncompatibleRappList*) - rapps filtered from the startup list for various reasons [latched]
        * **~parameters** (*std_msgs.String*) - displays the parameters used for this instantiation [latched]

        **Services**

        * **~list_rapps** (rapp_manager_srvs.GetRappList*) - service equivalent of the rapp_list topic
        * **~start_rapp** (rapp_manager_srvs.StartRapp*) - service to start a rapp
        * **~stop_rapp** (rapp_manager_srvs.StopRapp*) - service to stop a rapp

        **Parameters**

        :ivar current_rapp: currently running rapp, otherwise None
        :vartype current_rapp: rocon_app_manager.rapp.Rapp
        :ivar rocon_uri: this platform's signature used to determine if a rapp is compatible or not
        :vartype rocon_uri: str

        :ivar runnable_apps: these apps pass all filters and can be run
        :vartype runnable_apps:
        :ivar installable_apps: if the *auto_rapp_installation* parameter is set, then these rapps can be installed
        :vartype installable_apps:
        :ivar noninstallable_apps: if the *auto_rapp_installation* parameter is set, but dependencies aren't met, it populates this list
        :vartype noninstallable apps:
        :ivar platform_filtered_apps: rapps that don't match the platform that this rapp manager is running on
        :vartype platform_filtered_apps:
        :ivar capabilities_filtered_apps: rapps that don't have the required runtime capability support on this platform
        :vartype capabilities_filtered_apps:
        :ivar invalid_apps: rapps that are invalid for some reason, e.g. bad specification syntax
        :vartype invalid_apps:

        :ivar services: all ros services this instance provides
        :vartype services: rocon_python_comms.Services
        :ivar publishers: all ros publishers this instance provides
        :vartype publishers: rocon_python_comms.Publishers
        :ivar indexer: indexes all rapps on the system
        :vartype indexer: rocon_app_utilities.RappIndexer
        :ivar parameters: all ros parameters for this instance organised in a dictionary
        :vartype parameters: dict of string:value pairs.
    '''

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self.current_rapp = None
        roslaunch.pmon._init_signal_handlers()
        self.parameters = StandaloneParameters()
        self.rocon_uri = rocon_uri.generate_platform_rocon_uri(self.parameters.robot_type, self.parameters.robot_name)
        rospy.loginfo("Rapp Manager : rocon_uri: %s" % self.rocon_uri)
        # attempt to parse it, try and make it official
        try:
            rocon_uri.RoconURI(self.rocon_uri)
        except rocon_uri.exceptions.RoconURIValueError as e:
            rospy.logerr("Rapp Manager : rocon_uri is not official, rapp compatibility will probably fail [%s]" % str(e))
        self.caps_list = {}

        rospy.loginfo("Rapp Manager : indexing rapps...")
        self.indexer = rapp_repositories.get_combined_index(package_whitelist=self.parameters.rapp_package_whitelist, package_blacklist=self.parameters.rapp_package_blacklist)

        if self.parameters.auto_rapp_installation:
            try:
                self._dependency_checker = rocon_app_utilities.DependencyChecker()
                rospy.loginfo("Rapp Manager : auto rapp installation is enabled ..")
            except KeyError as unused_e:
                # TODO add a pointer to docs about configuring the environment for auto rapp installation
                rospy.logwarn("Rapp Manager : environment not configured for auto rapp installation, disabling.")
                self.parameters.auto_rapp_installation = False
        self.runnable_apps, self.installable_apps, self.noninstallable_apps, self.platform_filtered_apps, self.capabilities_filtered_apps, self.invalid_apps = self._determine_runnable_rapps()

        self._configure_preferred_rapp_for_virtuals()

        # ros communications
        self.services = rocon_python_comms.utils.Services(
            [
                ('~list_rapps', rapp_manager_srvs.GetRappList, self._process_get_runnable_rapp_list),
                ('~start_rapp', rapp_manager_srvs.StartRapp, self._process_start_rapp),
                ('~stop_rapp', rapp_manager_srvs.StopRapp, self._process_stop_rapp)
            ]
        )
        latched = True
        queue_size_five = 5
        self.publishers = rocon_python_comms.utils.Publishers(
            [
                ('~introspection/parameters', std_msgs.String, latched, queue_size_five),
                ('~status', rapp_manager_msgs.Status, latched, queue_size_five),
                ('~rapp_list', rapp_manager_msgs.RappList, latched, queue_size_five),
                ('~introspection/incompatible_rapp_list', rapp_manager_msgs.IncompatibleRappList, latched, queue_size_five)
            ]
        )
        # small pause (convenience only) to let connections to come up
        rospy.rostime.wallsleep(0.5)
        self.publishers.parameters.publish(std_msgs.String("%s" % self.parameters))
        self._publish_status()
        self._publish_rapp_list()
        self.publishers.incompatible_rapp_list.publish([], [], self.platform_filtered_apps, self.capabilities_filtered_apps)

        # TODO: Currently blacklisted apps and non-whitelisted apps are not provided yet

        if self.parameters.auto_start_rapp:  # None and '' are both false here
            request = rapp_manager_srvs.StartRappRequest()
            request.name = self.parameters.auto_start_rapp
            unused_response = self._process_start_rapp(request)

    def _init_capabilities(self):
        try:
            self.caps_list = CapsList()
        except exceptions.NotFoundException as e:
            if 'Timed out' in str(e):
                rospy.loginfo("Rapp Manager : disabling rapps requiring capabilities [server timed out]")
            elif "Couldn't find" in str(e):
                rospy.loginfo("Rapp Manager : disabling rapps requiring capabilities [server not found]")
            else:
                rospy.logwarn("Rapp Manager : disabling rapps requiring capabilities [%s]" % str(e))
            return False
        return True

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

    def _determine_runnable_rapps(self):
        '''
         Prune unsupported apps due to lack of support for required capabilities.

         :returns: incompatible app list dictionaries for capability incompatibilities respectively
         :rtype: {rocon_app_manager.Rapp}, [str], [str]
        '''
        compatible_rapps, platform_incompatible_rapps, invalid_rapps = self.indexer.get_compatible_rapps(uri=self.rocon_uri, ancestor_share_check=False)
        runnable_rapp_specs, capabilities_incompatible_rapps = self._filter_capability_unavailable_rapps(compatible_rapps)

        if self.parameters.auto_rapp_installation:
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
            rospy.logwarn("Rapp Manager : '" + str(rapp.resource_name) + "' is incompatible [" + rapp.raw_data['compatibility'] + "][" + self.rocon_uri + "]")

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
        full_apps.update(self.runnable_apps)
        full_apps.update(self.installable_apps)

        for unused_name, rapp in full_apps.items():
            ancestor_name = rapp.data['ancestor_name']
            if ancestor_name not in virtual_apps.keys():
                virtual_apps[ancestor_name] = []
                virtual_apps[ancestor_name].append(rapp)
            else:
                virtual_apps[ancestor_name].append(rapp)

        # Get preferred rapp configurations from parameter server and use
        v_rapps = {}

        for rapp_name, available_rapps in virtual_apps.items():
            if rapp_name not in self.parameters.preferred:
                if len(available_rapps) > 1:
                    v_rapps[rapp_name] = available_rapps[0]
                    rospy.logwarn("Rapp Manager : '" + rapp_name + "' is not unique and has no preferred rapp. " + available_rapps[0].data['name'] + "' has been selected.")
                else:
                    v_rapps[rapp_name] = available_rapps[0]
                continue
            else:
                preferred_rapp_name = self.parameters.preferred[rapp_name]

                if preferred_rapp_name not in full_apps:
                    rospy.logwarn("Rapp Manager : given preferred rapp '" + preferred_rapp_name + "' for '" + rapp_name + "' does not exist. '" + available_rapps[0].data['name'] + "' has been selected.")
                    v_rapps[rapp_name] = available_rapps[0]
                else:
                    rospy.loginfo("Rapp Manager : '%s' <- '%s' preferred." % (rapp_name, preferred_rapp_name))
                    v_rapps[rapp_name] = full_apps[preferred_rapp_name]
        self._virtual_apps = v_rapps

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

    ##########################################################################
    # Ros Callbacks
    ##########################################################################

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
            if name in self.parameters.preferred:
                rapp.preferred = self.parameters.preferred[name]

        for name, rapp in self.runnable_apps.items():
            ancestor_name = rapp.data['ancestor_name']
            avail[ancestor_name].implementations.append(name)

        for name, rapp in self.installable_apps.items():
            ancestor_name = rapp.data['ancestor_name']
            avail[ancestor_name].implementations.append(name)

        return avail.values()

    def _process_get_runnable_rapp_list(self, req):
        response = rapp_manager_srvs.GetRappListResponse()
        response.available_rapps = self._get_available_rapp_list()
        response.running_rapps = []
        if self.current_rapp:
            response.running_rapps.append(self.current_rapp.to_msg())
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
        if self.current_rapp:
            try:
                published_interfaces = self.current_rapp.published_interfaces_to_msg_list()
                published_parameters = self.current_rapp.published_parameters_to_msg_list()
                rapp = self.current_rapp.to_msg()
                rapp_status = rapp_manager_msgs.Status.RAPP_RUNNING
            except AttributeError:  # i.e. current_rapp is None
                # catch when self.current_rapp is NoneType since there is a miniscule chance
                # it might have changed inbetween the if check and the method calls
                pass  # nothing to do here as we predefine everything for this case.
        msg = rapp_manager_msgs.Status(application_namespace=self.parameters.application_namespace,
                                       rapp_status=rapp_status,
                                       rapp=rapp,
                                       published_interfaces=published_interfaces,
                                       published_parameters=published_parameters
                                       )
        try:
            self.publishers.status.publish(msg)
        except rospy.ROSException:  # publisher has unregistered - ros shutting down
            pass

    def _publish_rapp_list(self):
        '''
          Publishes an updated list of available apps (in that order).
        '''
        rapp_list = rapp_manager_msgs.RappList()
        try:
            rapp_list.available_rapps = self._get_available_rapp_list()
            self.publishers.rapp_list.publish(rapp_list)
        except rospy.exceptions.ROSException:  # publishing to a closed topic.
            pass

    def _process_start_rapp(self, req):
        resp = rapp_manager_srvs.StartRappResponse()
        resp.application_namespace = self.parameters.application_namespace
        rospy.loginfo("Rapp Manager : request received to start rapp [%s]" % req.name)

        # check is the app is already running
        if self.current_rapp:
            resp.started = False
            resp.message = "an app is already running [%s]" % self.current_rapp.data['name']
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
            (("' underneath '" + self.parameters.application_namespace + "'") if self.parameters.application_namespace else "'")
        )

        launch_arg_mappings = utils.LaunchArgMappings()
        launch_arg_mappings.application_namespace = self.parameters.application_namespace
        launch_arg_mappings.robot_name = self.parameters.robot_name
        launch_arg_mappings.rocon_uri = self.rocon_uri
        launch_arg_mappings.capability_nodelet_manager_name = caps_list.nodelet_manager_name if self.caps_list else None

        resp.started, resp.message, unused_connections = rapp.start(launch_arg_mappings,
                                                                    req.remappings,
                                                                    req.parameters,
                                                                    self.parameters.screen,
                                                                    self.caps_list
                                                                    )
        if resp.started:
            self.current_rapp = rapp
            self._publish_rapp_list()
            self._publish_status()
            thread.start_new_thread(self._monitor_rapp, ())
        return resp

    def _process_stop_rapp(self, req=None):
        '''
          Stops a currently running rapp. This can be triggered via the stop_app service call (in which
          case req is configured), or if the rapp monitoring thread detects that it has
          naturally stopped by itself (in which case req is None).

          :param req: variable configured when triggered from the service call.
        '''
        resp = rapp_manager_srvs.StopRappResponse()
        try:  # use a try-except block here so the check is atomic
            rapp_name = self.current_rapp.data['name']
            self.current_rapp = None  # immediately prevent it from trying to stop the app again
        except AttributeError:
            resp.stopped = False
            resp.error_code = rapp_manager_msgs.ErrorCodes.RAPP_IS_NOT_RUNNING
            resp.message = "tried to stop a rapp, but no rapp found running."
            rospy.logwarn("Rapp Manager : %s" % resp.message)
            return resp

        rospy.loginfo("Rapp Manager : stopping rapp '" + rapp_name + "'.")

        success, reason, rapp = self._check_runnable(rapp_name)
        if not success:
            resp.started = False
            resp.error_code = rapp_manager_msgs.ErrorCodes.RAPP_IS_NOT_AVAILABLE
            resp.message = reason
            rospy.logwarn("Rapp Manager : %s" % resp.message)
            return resp

        resp.stopped, resp.message, unused_connections = rapp.stop()

        if resp.stopped:
            if 'required_capabilities' in rapp.data:
                rospy.loginfo("Rapp Manager : stopping required capabilities.")
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
         yet or not. This gets run in a background thread when starting a rapp.
        '''
        while self.current_rapp:  # can be unset if stop_app service was directly called
            if not self.current_rapp.is_running():
                self._process_stop_rapp()
                break
            time.sleep(0.1)

    def _check_runnable(self, requested_rapp_name):
        success = False
        message = ""
        rapp = None

        if requested_rapp_name in self._virtual_apps.keys():  # Virtual rapp
            rapp = self._virtual_apps[requested_rapp_name]
            success = True
            message = ""

            if rapp.data['name'] in self.installable_apps.keys():
                success, message, rapp = self._install_rapp(rapp.data['name'])

        elif requested_rapp_name in self.runnable_apps.keys():  # Implementation Rapp
            rapp = self.runnable_apps[requested_rapp_name]
            success = True
            message = ""
        elif requested_rapp_name in self.installable_apps.keys():  # Installable Rapp
            success, message, rapp = self._install_rapp(requested_rapp_name)
        else:
            success = False
            message = ("The requested app '%s' is not among the runnable, nor installable rapps." % requested_rapp_name)
        return success, message, rapp

    def _install_rapp(self, requested_rapp_name):
        '''
        Try to install the specified rapp.

        :param str requested_rapp_name: name of the rapp to try and install
        :return: a triple (success, message, rapp)
        :rtype: (bool, str, rocon_app_manager.rapp.Rapp)
        '''
        success = False
        message = ""
        rapp = self.installable_apps[requested_rapp_name]

        if self.parameters.auto_rapp_installation:
            rospy.loginfo("Rapp Manager : Installing rapp '" + rapp.data['name'] + "'")
            success, reason = rapp.install(self._dependency_checker)
            if success:
                rospy.loginfo("Rapp Manager : Rapp '" + rapp.data['name'] + "'has been installed.")
                # move rapp from installable to runnable
                self.runnable_apps[requested_rapp_name] = rapp
                del self.installable_apps[requested_rapp_name]
                # TODO : consider calling publish_rapp_list if we split publishing runnable and installed there.

                success = True
                message = ""
            else:
                success = False
                message = "Installing rapp '" + rapp.data['name'] + "' failed. Reason: " + str(reason)
        else:
            success = False
            url = "'http://wiki.ros.org/rocon_app_manager/Tutorials/indigo/Automatic Rapp Installation'"
            message = str("Rapp '" + rapp.data['name'] + "' can be installed, " +
                          "but automatic installation is not enabled. Please refer to " + str(url) +
                          " for instructions on how to set up automatic rapp installation.")
        return success, message, rapp

    def spin(self):
        """
        A default spinner. Child classes will overload this with their own custom spinners.
        """
        rospy.spin()
