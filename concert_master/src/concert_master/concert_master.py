#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_master/LICENSE 
#

import roslib; roslib.load_manifest('concert_master')
import rospy
import sys, traceback

from gateway_msgs.srv import GatewayInfo
from rocon_gateway_hubclient.hub_client import HubClient

class ConcertMaster(object):
    concertmaster_key = "concertmasterlist"
    is_connected = False
    hub_client = None   

    gateway_info = '/gateway/gateway_info'
    gateway = None
  
    def __init__(self):
        self.is_connected = False
        self.name = rospy.get_name()
        self.param = self.setupRosParameters()

        is_zeroconf = False
        self.hub_client = HubClient(self.param['hub_whitelist'],self.param['hub_blacklist'],is_zeroconf,'rocon',self.name,False,None)

        self.service= {}
        self.service['gateway_info'] = rospy.ServiceProxy(self.gateway_info,GatewayInfo)


    def spin(self):
        self.log("Attempt to connect to Hub...")
        self.connectToHub()
        self.log("Connected to Hub [%s]"%self.hub_uri)
        self.log("Registering Concert Master...")
        self.registerConcertMaster()
        self.log("Concert Master Registered [%s]"%self.name)
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

        return param

    def _shutdown(self):
        rospy.loginfo("Shutting down")

    def log(self,msg):
        rospy.loginfo("Concert Master : " + msg)
    def logerr(self,msg):
        rospy.logerr("Concert Master : " + msg)

