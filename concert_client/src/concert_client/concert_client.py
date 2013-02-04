#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_client/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy

import rocon_app_manager_msgs.srv as rocon_app_manager_srvs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
from rocon_utilities import create_gateway_rule, create_gateway_remote_rule

##############################################################################
# Concert Client
##############################################################################


class ConcertClient(object):
    concertmaster_key = "concertmasterlist"

    hub_client = None

    gateway = None
    gateway_srv = {}

    concertmasterlist = []

    def __init__(self):
        ####################
        # Variables
        ####################
        self.name = rospy.get_name()
        self._param = self._setup_ros_parameters()

        ####################
        # Ros Api Handles
        ####################
        self.gateway_srv = {}
        self.gateway_srv['gateway_info'] = rospy.ServiceProxy('~gateway_info', gateway_srvs.GatewayInfo)
        try:
            self.gateway_srv['gateway_info'].wait_for_service()
        except rospy.exceptions.ROSInterruptException:
            rospy.logerr("Concert Client : interrupted while waiting for gateway services to appear.")
            return

        self.rocon_app_manager_srv = {}
        self.rocon_app_manager_srv['init'] = rospy.ServiceProxy('~init', rocon_app_manager_srvs.Init)
        self.rocon_app_manager_srv['init'].wait_for_service()

    def spin(self):
        gateway_info = gateway_srvs.GatewayInfoResponse()
        gateway_info.connected = False
        while not rospy.is_shutdown() and not gateway_info.connected:
            gateway_info = self.gateway_srv['gateway_info']()
            if gateway_info.connected:
                self.init(gateway_info.name)
            else:
                rospy.loginfo("Concert Client : gateway not yet connected,")
            rospy.sleep(1.0)
        rospy.spin()

    def init(self, name):
        '''
        @param name : the unique gateway name
        @type string
        @param uri : the hub uri
        @type string
        '''
        self.name = name

        app_init_req = rocon_app_manager_srvs.InitRequest(name)
        rospy.loginfo("Concert Client : initialising the app manager [%s]" % name)
        unused_resp = self.rocon_app_manager_srv['init'](app_init_req)

    def _setup_ros_parameters(self):
        param = {}
        return param
