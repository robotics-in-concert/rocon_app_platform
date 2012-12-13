#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_client/LICENSE
#

import roslib
roslib.load_manifest('concert_client')
import rospy
import sys, traceback
from .util import *

from rocon_hub_client.hub_client import HubClient
from .concertmaster_discovery import ConcertMasterDiscovery
from concert_msgs.srv import Invitation,Status,StatusResponse
from appmanager_msgs.srv import *


class ConcertClient(object):
    concertmaster_key = "concertmasterlist"

    is_connected = False
    is_invited = False
    hub_client = None

    gateway = None
    gateway_srv = {}

    concertmasterlist = []

    invitation_srv = 'invitation'
    status_srv = 'status'

    def __init__(self):
        self.is_connected = False
        self.name = rospy.get_name()
        self.param = self.setupRosParameters()

        self.hub_client = HubClient(whitelist=self.param['hub_whitelist'],
                                    blacklist=self.param['hub_blacklist'],
                                    is_zeroconf=False,
                                    namespace='rocon',
                                    name=self.name,
                                    callbacks=None)

        self.masterdiscovery = ConcertMasterDiscovery(self.hub_client, self.concertmaster_key, self.processNewMaster, self.log, self.logerr)

        self.gateway_srv= {}
        self.gateway_srv['gateway_info'] = rospy.ServiceProxy('/gateway/gateway_info',GatewayInfo)
        self.gateway_srv['flip'] = rospy.ServiceProxy('/gateway/flip',Remote)
        self.gateway_srv['gateway_info'].wait_for_service()
        self.gateway_srv['flip'].wait_for_service()

        self.appmanager_srv = {}
        self.appmanager_srv['init'] = rospy.ServiceProxy('/appmanager/init',Init)
        self.appmanager_srv['apiflip_request'] = rospy.ServiceProxy('/appmanager/apiflip_request',FlipRequest)
        self.appmanager_srv['invitation'] = rospy.ServiceProxy('/appmanager/invitation',Invitation)
        self.log("Wait for appmanager")
        self.appmanager_srv['init'].wait_for_service()

    def spin(self):
        self.log("Attempt to connect to Hub...")
        self.connectToHub()
        self.log("Connected to Hub [%s]"%self.hub_uri)
        self.log("Starting Master discovery thread...")
        self.startMasterDiscovery()
        rospy.spin()
        self.log("Stopping master discovery thread...")
        self.leaveMasters()
        self.log("Done")

    def connectToHub(self):
        while not rospy.is_shutdown() and not self.is_connected:
            self.log("Getting Hub info from gateway...")

            gateway_info = self.gateway_srv['gateway_info']()
            if gateway_info.connected == True:
                hub_uri = gateway_info.hub_uri
                if self.hub_client.connect(hub_uri):
                    self.init(gateway_info.name,hub_uri)
            else:
                self.log("No hub is available. Try later")
            rospy.sleep(1.0)

    def init(self,name,uri):
        self.is_connected = True
        self.name = name
        self.hub_uri = uri

        self.service = {}
        self.service['invitation'] = rospy.Service(self.name+'/'+self.invitation_srv,Invitation,self.processInvitation)
        self.service['status'] = rospy.Service(self.name+'/'+self.status_srv,Status,self.processStatus)
        self.master_services = ['/'+self.name+ '/'+self.invitation_srv , '/'+self.name + '/' + self.status_srv]

        app_init_req = InitRequest(name)
        resp = self.appmanager_srv['init'](app_init_req)
        self.log("Appmanager Initialization : " + str(resp))


    def setupRosParameters(self):
        param = {}
        param['hub_whitelist'] = ''
        param['hub_blacklist'] = ''
        param['cm_whitelist']= []
        param['cm_blacklist']= []

        return param

    def startMasterDiscovery(self):
        self.masterdiscovery.start()

    def processNewMaster(self,discovered_masterlist):
        # find newly discovered masters
        new_masters = [m for m in discovered_masterlist if m not in self.concertmasterlist]
        self.concertmasterlist += new_masters
        
        for master in new_masters:
            self.joinMaster(master)

        # cleaning gone masters
        self.concertmasterlist = [m for m in self.concertmasterlist and discovered_masterlist]

    def joinMaster(self,master):
        self.flips(master,self.master_services,ConnectionType.SERVICE,True)
        
        req = FlipRequestRequest(master,True)
        resp = self.appmanager_srv['apiflip_request'](req)
        if resp.result == False:
            self.logerr("Failed to Flip Appmanager APIs")


    def leaveMasters(self):
        self.masterdiscovery.setStop()

        try:
            for master in self.concertmasterlist:
                self.leavMaster(master)
        except Exception as e:
            self.log("Gateway is down already")

    def leavMaster(self,master):
        self.flips(master,self.master_services,ConnectionType.SERVICE,False)
        req = FlipRequestRequest(master,False)
        resp = self.appmanager_srv['apiflip_request'](req)
        if resp.result == False:
            self.logerr("Failed to Flip Appmanager APIs")




    def processInvitation(self,req):
        cm_name = req.name

        # Check if concert master is in white list
        if cm_name in self.param['cm_whitelist']: 
            return self.acceptInvitation(req)
        elif len(self.param['cm_whitelist']) == 0 and cm_name not in self.param['cm_blacklist']:
            return self.acceptInvitation(req)
        else :
            return InvitationResponse(False) 

    def acceptInvitation(self,req):
        self.log("Accepting invitation from " + req.name)
        resp = self.appmanager_srv['invitation'](req)
        
        return resp

    def processStatus(self,req):
        resp = StatusResponse()
        resp.status = "Vacant" if not self.is_invited else "Busy"
        return resp

    def flips(self,remote_name,topics,type,ok_flag):
        if len(topics) == 0:
            return
        req = RemoteRequest()
        req.cancel = not ok_flag
        req.remotes = []
        for t in topics:
            req.remotes.append(createRemoteRule(remote_name,createRule(t,type)))

        resp = self.gateway_srv['flip'](req) 

        if resp.result == 0:
            self.log("Success to Flip : " + str(topics))
        else:
            self.logerr("Failed to flip : " + str(topics) + " : " + str(resp.error_message))

    def log(self, msg):
        rospy.loginfo("Concert Client : " + msg)

    def logerr(self, msg):
        rospy.logerr ("Concert Client : " + msg)
