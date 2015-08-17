#
# License: BSD
#   https://raw.github.com/robotics-in-py/rocon_app_platform/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: py
   :platform: Unix
   :synopsis: A rapp manager class for connection with subserviant concerts.


A simpler app manager to use for connections to concerts than that used for
rocon indigo.

----

"""

##############################################################################
# Imports
##############################################################################

import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_gateway_utils
import rocon_python_comms
import rospy
import std_msgs.msg as std_msgs

# local imports
from .exceptions import GatewayNotFoundException
from .standalone import Standalone
from .ros_parameters import ConcertParameters

##############################################################################
# Rapp Manager
##############################################################################


class ConcertClient(Standalone):
    '''
    A rapp manager for use with subserviant concerts.

    **Features**

    * **application namespace** - communicates with a gateway to set a unique application namespace in /py/clients/_name_
    * **firing the gateway** - pings the gateway whenever a rapp is started to create low latency flips

    **Publishers**

    * **~force_update** (*rapp_manager_msgs.IncompatibleRappList*) - rapps filtered from the startup list for various reasons [latched]

    .. todo:: if we want low latency flipping, call the gateway watcher set period service with low period after starting a rapp
    '''

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        super(ConcertClient, self).__init__()

        # parameters
        self.concert_parameters = ConcertParameters()

        # working with the gateway
        try:
            self._match_robot_name_to_gateway_name()
            self._set_gateway_flip_rules()
        except GatewayNotFoundException:
            return  # error messages already in the underlying functions

        latched = True
        queue_size_five = 5
        self.concert_publishers = rocon_python_comms.utils.Publishers(
            [
                ('~concert_parameters', std_msgs.String, latched, queue_size_five),
                ('/concert/clients/' + self.parameters.robot_name + '/rocon_uri', std_msgs.String, latched, queue_size_five),
            ]
        )
        rospy.rostime.wallsleep(0.5)  # let the publisher come up
        self.concert_publishers.concert_parameters.publish(std_msgs.String("%s" % self.concert_parameters))
        self.concert_publishers.rocon_uri.publish(std_msgs.String(self.rocon_uri))

        # done
        rospy.loginfo("Rapp Manager : initialised and ready for concert interactions")

    def _match_robot_name_to_gateway_name(self):
        gateway_info_service = rocon_python_comms.SubscriberProxy('~gateway_info', gateway_msgs.GatewayInfo)
        rate = rospy.Rate(10)  # 10hz
        warning_throttle_counter = 0
        while not rospy.is_shutdown():
            gateway_info = gateway_info_service(timeout=rospy.Duration(0.3))
            if gateway_info:
                if self.parameters.robot_name != gateway_info.name.lower().replace(' ', '_'):
                    self.parameters.robot_name = gateway_info.name.lower().replace(' ', '_')
                    rospy.loginfo("Rapp Manager : matching robot name with gateway name '%s'" % self.parameters.robot_name)
                break
            try:
                rate.sleep()
                warning_throttle_counter += 1
                if warning_throttle_counter % 10 == 0:
                    rospy.logerr("Rapp Manager : unable to find the local gateway.")
            except rospy.exceptions.ROSInterruptException:
                rospy.loginfo("Rapp Manager : breaking out of gateway search loop [most likely just ros shutting down]")
                raise GatewayNotFoundException()

    def _set_gateway_flip_rules(self, cancel_flag=False):
        req = gateway_srvs.RemoteRequest()
        req.cancel = cancel_flag
        flip_request_service = rospy.ServiceProxy('~flip', gateway_srvs.Remote)
        print("Resolved : %s" % flip_request_service.resolved_name)

        concert_namespace_rule = rocon_gateway_utils.create_gateway_rule(name="/concert/.*", connection_type=gateway_msgs.ConnectionType.PUBLISHER)
        applications_namespace_rule = rocon_gateway_utils.create_gateway_rule(name=self.parameters.application_namespace + "/.*", connection_type=gateway_msgs.ConnectionType.PUBLISHER)
        for concert_remote_gateway in self.concert_parameters.concert_whitelist:
            req.remotes.append(rocon_gateway_utils.create_gateway_remote_rule(concert_remote_gateway, concert_namespace_rule))
            req.remotes.append(rocon_gateway_utils.create_gateway_remote_rule(concert_remote_gateway, applications_namespace_rule))
        if not self.concert_parameters.concert_whitelist:
            req.remotes.append(rocon_gateway_utils.create_gateway_remote_rule(".*", concert_namespace_rule))
            req.remotes.append(rocon_gateway_utils.create_gateway_remote_rule(".*", applications_namespace_rule))

        warning_throttle_counter = 0
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            try:
                resp = flip_request_service(req)
            except rospy.service.ServiceException:
                raise GatewayNotFoundException()  # often disappears when the gateway shuts down just before the app manager, ignore silently.

            if resp.result == gateway_msgs.ErrorCodes.SUCCESS:
                rospy.loginfo("Rapp Manager : set flip rules on the local gateway [remotes: %s]" % self.concert_parameters.concert_whitelist)
                break
            else:
                if resp.result == gateway_msgs.ErrorCodes.NO_HUB_CONNECTION and cancel_flag:
                    # can often happen if a concert goes down and brings the hub down as as well
                    # so just suppress this warning if it's a request to cancel
                    rospy.logwarn("Rapp Manager : failed to cancel flips (probably remote hub intentionally went down as well) [%s, %s]" % (resp.result, resp.error_message))
                    raise GatewayNotFoundException()  # often disappears when the gateway shuts down just before the app manager, ignore silently.
                else:
                    warning_throttle_counter += 1
                    if warning_throttle_counter % 10 == 0:
                        rospy.logerr("Rapp Manager : could not set flip rules on the local gateway")
                rate.sleep()

    ##########################################################################
    # Execute
    ##########################################################################

    def spin(self):
        """
        A default spinner. Child classes will overload this with their own custom spinners.
        """
        rospy.spin()
