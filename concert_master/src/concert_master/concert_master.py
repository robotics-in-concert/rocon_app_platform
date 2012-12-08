#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_master/LICENSE 
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

    gateway_info = '/gateway/gateway_info'
    gateway = None

    set_auto_invite = '/concertconductor/set_auto_invite'
  
    def __init__(self):
        self.is_connected = False
        is_zeroconf = False

        self.name = rospy.get_name()
        self.param = self.setupRosParameters()

        self.hub_client = HubClient(self.param['hub_whitelist'],self.param['hub_blacklist'],is_zeroconf,'rocon',self.name,False,None)

        self.service= {}
        self.service['gateway_info'] = rospy.ServiceProxy(self.gateway_info,GatewayInfo)
        self.service['gateway_info'].wait_for_service()

        self.service['set_auto_invite'] = rospy.ServiceProxy(self.set_auto_invite,SetAutoInvite)
        self.service['set_auto_invite'].wait_for_service()


    def spin(self):
        self.log("Attempt to connect to Hub...")
        self.connectToHub()
        self.log("Connected to Hub [%s]"%self.hub_uri)
        self.log("Registering Concert Master...")
        self.registerConcertMaster()
        self.log("Concert Master Registered [%s]"%self.name)
        
        if self.param['auto_invite']:
            self.setAutoInvite()

        rospy.spin()
        self.log("Shutting Down")
        self.shutdown()
        self.log("Done")

    def connectToHub(self):
        while not rospy.is_shutdown() and not self.is_connected:
            self.log("Getting Hub info from gateway...")

            gateway_info = self.service['gateway_info']()
            if gateway_info.connected == True:
                hub_uri = gateway_info.hub_uri
                if self.hub_client.connect(hub_uri):
                    self.is_connected = True
                    self.name = gateway_info.name
                    self.hub_uri = hub_uri
            else:
                self.log("No hub is available. Try later")
            rospy.sleep(1.0)

    def setAutoInvite(self):
        try:
            req = SetAutoInviteRequest(self.name,self.param['auto_invite'])
            self.service['set_auto_invite'](req)
        except Exception as e:
            self.logerr("Failed to call [set_auto_invite] : " + str(e))
            

    def registerConcertMaster(self):
        try:
            self.hub_client.registerKey(self.concertmaster_key,self.name)
        except Exception as e:
            self.logerr(str(e))
            traceback.print_exc(file=sys.stdout)
            

    def shutdown(self):
        self.unregisterConcertMaster()

    def unregisterConcertMaster(self):
        self.hub_client.unregisterKey(self.concertmaster_key,self.name)


    def setupRosParameters(self):
        param = {}
        param['hub_uri'] = rospy.get_param('~hub_uri','')
        param['hub_whitelist'] = rospy.get_param('~hub_whitelist','')
        param['hub_blacklist'] = rospy.get_param('~hub_blacklist','')
        param['auto_invite'] = rospy.get_param('~auto_invite',False)

        return param

    def _shutdown(self):
        rospy.loginfo("Shutting down")

    def log(self,msg):
        rospy.loginfo("Concert Master : " + msg)
    def logerr(self,msg):
        rospy.logerr("Concert Master : " + msg)

