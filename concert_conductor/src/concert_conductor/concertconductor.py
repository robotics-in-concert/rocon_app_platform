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
    invited_clients = {}
    mastername = None

    def __init__(self):
        self.srv = {}
        self.srv['clientlist'] = rospy.Service('~list_clients',ClientList,self.processClientList)
        self.srv['invite'] = rospy.Service('~invite',Invite,self.processInvite)
        self.srv['set_auto_invite'] = rospy.Service('~set_auto_invite',SetAutoInvite,self.processAutoInvite)

        self.parse_params()

    def parse_params(self):
        param = {}
        param['config'] = {}
        param['config']['auto_invite'] = rospy.get_param('~auto_invite',False)

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
        clients = [s[:-(len(suffix)+1)] for s in services if s.endswith(suffix)]

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

        resp = self.invite(mastername,req.clientnames,req.ok_flag)

        return InviteResponse("Success to invite["+str(resp)+"] : " + str(req.clientnames))

    def processAutoInvite(self,req):
        self.log("Auto Invitation : " + str(req.is_auto))
        self.mastername = req.mastername
        self.param['config']['auto_invite'] = req.is_auto 
        return SetAutoInviteResponse(True)

    def invite(self,mastername,clientnames,ok_flag):
        try:
            for name in clientnames:
                if not name.startswith('/'):
                    name = '/'+ name
                resp = self.clients[name].invite(mastername,ok_flag)
                self.log("Success to invite["+str(ok_flag)+"] : " + str(name))
                self.invited_clients[name] = ok_flag
        except Exception as e:
            self.logerr(str(e))
            return False

        return True
                                                                                    

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

            if self.param['config']['auto_invite']:
                client_list = [ client for client in self.clients if (client not in self.invited_clients) or (client in self.invited_clients and self.invited_clients[client] == False)]
                self.invite(self.mastername,client_list,True)

            rospy.sleep(1.0)
    

    def log(self,msg):
        rospy.loginfo("Concert Conductor : " + msg)

    def logerr(self,msg):
        rospy.logerr("Concert Conductor : " + msg)


