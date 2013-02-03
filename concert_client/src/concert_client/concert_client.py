#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_client/LICENSE
#
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('concert_client')
import rospy

from rocon_hub_client.hub_client import HubClient
from .concertmaster_discovery import ConcertMasterDiscovery
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs
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

    invitation_srv = 'invitation'

    def __init__(self):
        ####################
        # Variables
        ####################
        self._is_connected_to_hub = False
        self.name = rospy.get_name()
        self.param = self._setup_ros_parameters()

        self.hub_client = HubClient(whitelist=self.param['hub_whitelist'],
                                    blacklist=self.param['hub_blacklist'],
                                    is_zeroconf=False,
                                    namespace='rocon',
                                    name=self.name,
                                    callbacks=None)

        self.masterdiscovery = ConcertMasterDiscovery(self.hub_client, self.concertmaster_key, self.processNewMaster)
        self._is_connected_to_concert = False
        self._is_connected_to_hub = False

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
        self.connectToHub()
        rospy.loginfo("Concert Client: connected to Hub [%s]" % self.hub_uri)
        rospy.loginfo("Concert Client; scanning for concerts...")
        self.masterdiscovery.start()
        rospy.spin()
        self.masterdiscovery.set_stop()

    def connectToHub(self):
        while not rospy.is_shutdown() and not self._is_connected_to_hub:
            rospy.loginfo("Getting Hub info from gateway...")
            gateway_info = self.gateway_srv['gateway_info']()
            if gateway_info.connected == True:
                hub_uri = gateway_info.hub_uri
                if self.hub_client.connect(hub_uri):
                    self.init(gateway_info.name, hub_uri)
            else:
                rospy.loginfo("No hub is available. Try later")
            rospy.sleep(1.0)

    def init(self, name, uri):
        '''
        @param name : the unique gateway name
        @type string
        @param uri : the hub uri
        @type string
        '''
        self._is_connected_to_hub = True
        self.name = name
        self.hub_uri = uri

        self.master_services = ['/' + self.name + '/' + self.invitation_srv]

        app_init_req = rocon_app_manager_srvs.InitRequest(name)
        rospy.loginfo("Concert Client : initialising the app manager [%s]" % name)
        unused_resp = self.rocon_app_manager_srv['init'](app_init_req)

    def _setup_ros_parameters(self):
        param = {}
        param['hub_whitelist'] = ''
        param['hub_blacklist'] = ''

        return param

    def processNewMaster(self, discovered_masterlist):
        # find newly discovered masters
        new_masters = [m for m in discovered_masterlist if m not in self.concertmasterlist]
        self.concertmasterlist += new_masters

        # cleaning gone masters
        self.concertmasterlist = [m for m in self.concertmasterlist and discovered_masterlist]
