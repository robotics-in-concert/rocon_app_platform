#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_orchhestration/master/concert_master/LICENSE
#

import roslib
roslib.load_manifest('concert_master')
import rospy
import sys
import traceback

from concert_msgs.srv import *
from gateway_msgs.srv import GatewayInfo
from rocon_hub_client.hub_client import HubClient


class ConcertMaster(object):
    concertmaster_key = "concertmasterlist"
    is_connected = False
    hub_client = None

    def __init__(self):
        self.is_connected = False

        self.name = rospy.get_name()
        self.param = self._setup_ros_parameters()

        self.hub_client = HubClient(whitelist=self.param['hub_whitelist'],
                                    blacklist=self.param['hub_blacklist'],
                                    is_zeroconf=False,
                                    namespace='rocon',
                                    name=self.name,
                                    callbacks=None)

        self.service = {}
        try:
            self.service['gateway_info'] = rospy.ServiceProxy("~gateway_info", GatewayInfo)
            self.service['gateway_info'].wait_for_service()
            self.service['set_auto_invite'] = rospy.ServiceProxy('~set_auto_invite', SetAutoInvite)
            self.service['set_auto_invite'].wait_for_service()
        except rospy.exceptions.ROSInterruptException:
            rospy.logwarn("Concert Master : ros shut down before services could be found.")

    def spin(self):
        self._connect_to_hub()
        rospy.loginfo("Concert Master : connected to hub [%s]" % self.hub_uri)
        self._register_concert_master()
        rospy.loginfo("Concert Master : registered on the hub [%s]" % self.name)

        if self.param['auto_invite']:
            self._set_auto_invite()

        rospy.spin()
        self._shutdown()

    def _connect_to_hub(self):
        while not rospy.is_shutdown() and not self.is_connected:
            gateway_info = self.service['gateway_info']()
            if gateway_info.connected == True:
                hub_uri = gateway_info.hub_uri
                if self.hub_client.connect(hub_uri):
                    self.is_connected = True
                    self.name = gateway_info.name
                    self.hub_uri = hub_uri
            else:
                rospy.loginfo("Concert Master : no hub yet available, spinning...")
            rospy.sleep(1.0)

    def _set_auto_invite(self):
        try:
            req = SetAutoInviteRequest(self.name, self.param['auto_invite'])
            self.service['set_auto_invite'](req)
        except Exception as e:
            rospy.logerr("Concert Master : failed to call [set_auto_invite] : " + str(e))

    def _setup_ros_parameters(self):
        param = {}
        param['hub_uri'] = rospy.get_param('~hub_uri', '')
        param['hub_whitelist'] = rospy.get_param('~hub_whitelist', '')
        param['hub_blacklist'] = rospy.get_param('~hub_blacklist', '')
        param['auto_invite'] = rospy.get_param('~auto_invite', False)

        return param

    def _register_concert_master(self):
        try:
            self.hub_client.registerKey(self.concertmaster_key, self.name)
        except Exception as e:
            rospy.logerr("Concert Master : %s" % str(e))
            traceback.print_exc(file=sys.stdout)

    def _unregister_concert_master(self):
        try:
            self.hub_client.unregisterKey(self.concertmaster_key, self.name)
        except HubClient.ConnectionError:
            # usually just shut down before we did...die quietly
            #rospy.logwarn("Concert Master : lost connection to the hub (probably shut down before we did)")
            pass

    def _shutdown(self):
        self._unregister_concert_master()
        rospy.loginfo("Concert Master : shutting down")
