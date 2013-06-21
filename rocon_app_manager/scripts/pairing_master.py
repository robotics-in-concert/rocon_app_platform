#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/hydro-devel/rocon_app_manager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import sys
import rospy
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs
import rocon_utilities
import rocon_utilities.console as console

##############################################################################
# Methods
##############################################################################

def local_gateway_name():
    
    gateway_name = None
    gateway_info_service = rocon_utilities.SubscriberProxy('~gateway_info', gateway_msgs.GatewayInfo)
    while not rospy.is_shutdown():
        gateway_info = gateway_info_service(timeout=rospy.Duration(0.2))
        if gateway_info:
            if gateway_info.connected:
                gateway_name = gateway_info.name
                break
        rospy.sleep(1.0)
    return gateway_name

def remote_gateway_name():
    '''
      Assumption: note that the remote gateway info in the paired master system
      should only ever show at most, one remote gateway. That should be the
      private counterpart (we're not using zeroconf over here).
      
      @return namespace of the app manager (matches the remote gateway name)
      @rtype string or None : string does not yet prefix a leading '/'
    '''
    remote_gateway_info_service = rospy.ServiceProxy('~remote_gateway_info', gateway_srvs.RemoteGatewayInfo)
    remote_gateway_name = None
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service('~remote_gateway_info', timeout=0.3)
        except rospy.exceptions.ROSException:  # timeout exceeded
            continue
        except (rospy.exceptions.ROSInterruptException, rospy.service.ServiceException):  # shutdown exception
            sys.exit(0)
        remote_gateway_info = remote_gateway_info_service()
        if len(remote_gateway_info.gateways) == 1:
            remote_gateway_name = remote_gateway_info.gateways[0].name
            break
        elif len(remote_gateway_info.gateways) == 2:
            console.error("Pairing Master : found two remote gateways when there should only ever be one.")
            sys.exit(1)
    if remote_gateway_name is None:
        # probably shutting down
        #console.error("Pairing Master : app_manager_namespace returned 'None', probably shutting down.")
        sys.exit(0)
    return remote_gateway_name

class InvitationHandler():
    
    __slots__ = ['local_gateway_name', 'remote_gateway_name', 'remote_invite_service', 'relay_invitation_server']
    
    def __init__(self, local_gateway_name, remote_gateway_name):
        self.local_gateway_name = local_gateway_name
        self.remote_gateway_name = remote_gateway_name
        remote_invite_service_name = '/' + remote_gateway_name + '/invite'
        self.remote_invite_service = rospy.ServiceProxy(remote_invite_service_name, rocon_app_manager_srvs.Invite)
        rospy.loginfo("Pairing Master : waiting for invitation service [%s]" % remote_invite_service_name)
        rospy.wait_for_service(remote_invite_service_name)
        rospy.loginfo("Pairing Master : initialising simple client invitation service [%s]" % remote_invite_service_name)
        self.relay_invitation_server = rospy.Service('/' + remote_gateway_name + '/pairing_mode_invite', rocon_app_manager_srvs.Invite, self.relayed_invitation)
        
    def relayed_invitation(self, req):
        '''
          Provides a relayed invitation from a client (e.g. android remocon).
          This relay fills in the gateway name to pass on to the app manager,
          which makes life much simpler for the clients.
        '''
        try:
            rospy.loginfo("Pairing Master : inviting the private master's application manager.")
            print("Local gateway name: %s" % local_gateway_name)
            remote_response = self.remote_invite_service(rocon_app_manager_srvs.InviteRequest(
                                                         remote_target_name=self.local_gateway_name,
                                                         application_namespace='',
                                                         cancel=req.cancel))
        except rospy.service.ServiceException:  # service call failed
            console.logerror("Pairing Master: remote invitation failed to connect.")
        except rospy.exceptions.ROSInterruptException:  # shutdown exception
            sys.exit(0)
        return remote_response
    
##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('pairing_master')
    auto_invite = rospy.get_param("~auto_invite", "false")
    rospy.loginfo("Pairing Master : auto-inviting %s" % auto_invite)
    local_gateway_name = local_gateway_name()
    if local_gateway_name is None:
        console.logerror("Pairing Master : shutting down.")
        sys.exit(1)
    rospy.loginfo("Pairing Master : local gateway name [%s]" % local_gateway_name)
    remote_gateway_name = remote_gateway_name()
    rospy.loginfo("Pairing Master : remote gateway name [%s]" % remote_gateway_name)

    invitation_handler = InvitationHandler(local_gateway_name, remote_gateway_name)
    rospy.spin()
