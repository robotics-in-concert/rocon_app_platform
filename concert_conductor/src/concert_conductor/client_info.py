#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_conductor/LICENSE
#

##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('concert_conductor')
import rospy
import concert_msgs.msg as concert_msgs
import concert_msgs.srv as concert_srvs

##############################################################################
# ClientInfo Class
##############################################################################


class ClientInfo(object):

    def __init__(self, clientname, param):

        self.data = concert_msgs.ConcertClient()
        self.rawdata = {}
        self.name = clientname
        self.param = param

        self.platform_info = None
        self.service_info = {}
        self.service_exec = {}

        #### Setup Invitation Service
        self.invitation = rospy.ServiceProxy(str(self.name + '/' + param['invitation'][0]), param['invitation'][1])

        self.service_info = {}
        for k in param['info'].keys():
            key = param['info'][k][0]
            type = param['info'][k][1]
            self.service_info[k] = rospy.ServiceProxy(str(self.name + '/' + key), type)
            print "Waiting for : " + str(k)
            try:
                self.service_info[k].wait_for_service()
            except rospy.ServiceException, e:
                raise e
        self.read_info()

    def read_info(self):
        try:
            for key, service in self.service_info.items():
                self.rawdata[key] = service()
        except Exception as unused_e:
            raise Exception("Error in read_info")

        self.data = concert_msgs.ConcertClient()
        self.data.name = self.name
        self.data.platform_info = str(self.rawdata['platform_info'].platform)
        self.data.status = self.rawdata['status'].status
        self.data.apps = self.rawdata['list_apps'].apps

    def get_client(self):
        self.read_info()
        return self.data

    def invite(self, name, ok_flag):
        req = concert_srvs.InvitationRequest(name, ok_flag)

        resp = self.invitation(req)

        if resp.success == True:
            self.set_channel()
        else:
            raise Exception(str("Invitation Failed : " + self.name))

    def set_channel(self):
        param = self.param
        # Services
        self.service_exec = {}
        for k in param['execution']['srv'].keys():
            key = param['execution']['srv'][k][0]
            type = param['execution']['srv'][k][1]
            self.service_exec[k] = rospy.ServiceProxy(str(self.name + '/' + key), type)
