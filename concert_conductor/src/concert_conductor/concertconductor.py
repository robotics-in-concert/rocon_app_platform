#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_conductor/LICENSE 
#

import roslib; roslib.load_manifest('concert_conductor')
import rospy
import rosservice
from appmanager_msgs.srv import *
from concert_msgs.srv import *
from std_msgs.msg import String
from .clientinfo import ClientInfo


class ConcertConductor(object):

    clients = {}

    def __init__(self):
        self.srv = {}
        self.srv['clientlist'] = rospy.Service('~clientlist',ClientList,self.processClientList)
        self.srv['invite'] = rospy.Service('~invite',Invite,self.processInvite)

        self.parse_params()

    def parse_params(self):
        param = {}
        param['invitation'] = (rospy.get_param('~invitation','invitation'), Invitation)
        param['info'] = {}
        param['info']['list_apps'] = (rospy.get_param('~list_apps','list_apps'),GetAppList)
        param['info']['platform_info'] = (rospy.get_param('~platform_info','platform_info'),PlatformInfoSrv)
        param['info']['status'] = (rospy.get_param('~status','status'),Status)

        param['execution'] = {}
        param['execution']['srv'] = {}
        param['execution']['srv']['start_app'] = (rospy.get_param('~start_app','start_app'),StartApp)
        param['execution']['srv']['stop_app'] = (rospy.get_param('~stop_app','stop_app'),StopApp)

        self.param = param

    def get_clients(self):
        suffix = self.param['invitation'][0]
        services = rosservice.get_service_list()
        clients = [ s[:-(len(suffix)+1)] for s in services if s.endswith(suffix)]

        return clients

    def maintain_clientlist(self,new_clients):
        for name in self.clients.keys():
            if not name in new_clients:
                self.log("Client Left : " + name)
                del self.clients[name]

    def processClientList(self,req):
        out = [ self.clients[cinfo].get_client() for cinfo in self.clients]
        
        return ClientListResponse(out)

    def processInvite(self,req):
        mastername = req.mastername
        try:
            for name in req.clientnames:
                if not name.startswith('/'):
                    name = '/'+ name
                self.clients[name].invite(mastername,req.ok_flag)
        except Exception as e:
            self.logerr(str(e))

        self.log("Success to invite["+str(req.ok_flag)+"] : " + str(req.clientnames))
        return InviteResponse("Success to invite["+str(req.ok_flag)+"] : " + str(req.clientnames))
                   

    def spin(self):
        while not rospy.is_shutdown():
            # Get all services and collect 
            clients = self.get_clients()

            # remove gone client
            self.maintain_clientlist(clients)

            # filter existing client from new client list
            new_clients = [c for c in clients if c not in self.clients]
            
            # Create new clients info instance
            for new_client in new_clients:
                self.log("Client Join : " + new_client) 
                cinfo = ClientInfo(new_client,self.param)
                self.clients[new_client] = cinfo

            rospy.sleep(1.0)
    

    def log(self,msg):
        rospy.loginfo("Concert Conductor : " + msg)

    def logerr(self,msg):
        rospy.logerr("Concert Conductor : " + msg)


