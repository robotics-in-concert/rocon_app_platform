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
    status_srv = 'status'

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
        self.gateway_srv['flip'] = rospy.ServiceProxy('~flip', gateway_srvs.Remote)
        try:
            self.gateway_srv['gateway_info'].wait_for_service()
            self.gateway_srv['flip'].wait_for_service()
        except rospy.exceptions.ROSInterruptException:
            rospy.logerr("Concert Client : interrupted while waiting for gateway services to appear.")
            return

        self.rocon_app_manager_srv = {}
        self.rocon_app_manager_srv['init'] = rospy.ServiceProxy('~init', rocon_app_manager_srvs.Init)
        self.rocon_app_manager_srv['init'].wait_for_service()
        self.rocon_app_manager_srv['invite'] = rospy.ServiceProxy('~invite', rocon_app_manager_srvs.Invite)
        self.rocon_app_manager_srv['invite'].wait_for_service()

    def spin(self):
        self.connectToHub()
        rospy.loginfo("Concert Client: connected to Hub [%s]" % self.hub_uri)
        rospy.loginfo("Concert Client; scanning for concerts...")
        self.masterdiscovery.start()
        rospy.spin()
        self.leaveMasters()

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

        self.service = {}
        self.service['invitation'] = rospy.Service(self.name + '/' + self.invitation_srv, concert_srvs.Invitation, self._process_invitation)
        self.service['status'] = rospy.Service(self.name + '/' + self.status_srv, concert_srvs.Status, self._process_status)
        self.master_services = ['/' + self.name + '/' + self.invitation_srv, '/' + self.name + '/' + self.status_srv]

        app_init_req = rocon_app_manager_srvs.InitRequest(name)
        rospy.loginfo("Concert Client : initialising the app manager [%s]" % name)
        unused_resp = self.rocon_app_manager_srv['init'](app_init_req)

    def _setup_ros_parameters(self):
        param = {}
        param['hub_whitelist'] = ''
        param['hub_blacklist'] = ''
        param['cm_whitelist'] = []
        param['cm_blacklist'] = []

        return param

    def processNewMaster(self, discovered_masterlist):
        # find newly discovered masters
        new_masters = [m for m in discovered_masterlist if m not in self.concertmasterlist]
        self.concertmasterlist += new_masters

        for master in new_masters:
            self.joinMaster(master)

        # cleaning gone masters
        self.concertmasterlist = [m for m in self.concertmasterlist and discovered_masterlist]

    def joinMaster(self, master):
        self.flips(master, self.master_services, gateway_msgs.ConnectionType.SERVICE, True)

    def leaveMasters(self):
        self.masterdiscovery.set_stop()
        try:
            for master in self.concertmasterlist:
                self._leave_master(master)
        except Exception as unused_e:
            rospy.logdebug("Concert Client: gateway already down, no shutdown work required.")

    def _leave_master(self, master):
        self.flips(master, self.master_services, gateway_msgs.ConnectionType.SERVICE, False)

    def _process_invitation(self, req):
        cm_name = req.name

        # Check if concert master is in white list
        if cm_name in self.param['cm_whitelist']:
            return self.acceptInvitation(req)
        elif len(self.param['cm_whitelist']) == 0 and cm_name not in self.param['cm_blacklist']:
            return concert_srvs.InvitationResponse(self.acceptInvitation(req))
        else:
            return concert_srvs.InvitationResponse(False)

    def acceptInvitation(self, req):
        rospy.loginfo("Concert Client : accepting invitation from %s" % req.name)
        app_manager_invite_request = rocon_app_manager_srvs.InviteRequest()
        app_manager_invite_request.remote_target_name = req.name
        app_manager_invite_request.cancel = not req.ok_flag
        resp = self.rocon_app_manager_srv['invite'](app_manager_invite_request)
        self._is_connected_to_concert = resp.result
        return resp.result

    def _process_status(self, req):
        resp = concert_srvs.StatusResponse()
        resp.status = "free-agent" if not self._is_connected_to_concert else "busy"
        resp.client_status = concert_msgs.Constants.CONCERT_CLIENT_STATUS_AVAILABLE \
                        if not self._is_connected_to_concert else concert_msgs.Constants.CONCERT_CLIENT_STATUS_CONCERT
        resp.app_status = concert_msgs.Constants.APP_STATUS_STOPPED  # This is just a stub for now https://github.com/robotics-in-concert/rocon_app_platform/issues/20
        return resp

    def flips(self, remote_name, topics, type, ok_flag):
        if len(topics) == 0:
            return
        req = gateway_srvs.RemoteRequest()
        req.cancel = not ok_flag
        req.remotes = []
        for t in topics:
            req.remotes.append(create_gateway_remote_rule(remote_name, create_gateway_rule(t, type)))

        resp = self.gateway_srv['flip'](req)

        if resp.result == 0:
            rospy.loginfo("Concert Client : successfully flipped to the concert %s" % str(topics))
        else:
            rospy.logerr("Concert Client : failed to flip [%s][%s]" % (str(topics), str(resp.error_message)))
