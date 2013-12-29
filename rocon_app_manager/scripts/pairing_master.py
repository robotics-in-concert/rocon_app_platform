#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/hydro-devel/rocon_app_manager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import sys
import rosgraph
import rospy
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs
import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_utilities
import rocon_utilities.console as console
import std_msgs.msg as std_msgs

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
        rospy.rostime.wallsleep(1.0)
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
    
    __slots__ = ['local_gateway_name', 'remote_gateway_name', 'remote_invite_service', 'relay_invitation_server', 'watchdog_flag'] 
    
    def __init__(self, local_gateway_name, remote_gateway_name, auto_invite):
        self.watchdog_flag = True
        self.local_gateway_name = local_gateway_name
        self.remote_gateway_name = remote_gateway_name
        self.auto_invite = auto_invite
        remote_invite_service_name = '/' + remote_gateway_name + '/invite'
        # Speed up the gateway watcher for convenience (otherwise it'll take like 5 seconds to pick up the app manager
        gateway_watcher_period_service = rospy.ServiceProxy('~set_watcher_period', gateway_srvs.SetWatcherPeriod)
        unused_watcher_period_response = gateway_watcher_period_service(gateway_srvs.SetWatcherPeriodRequest(0.25)) 
        # Detect the app manager
        self.remote_invite_service = rospy.ServiceProxy(remote_invite_service_name, rocon_app_manager_srvs.Invite)
        rospy.loginfo("Pairing Master : waiting for invitation service [%s]" % remote_invite_service_name)
        try:
            rospy.wait_for_service(remote_invite_service_name)
        except rospy.exceptions.ROSInterruptException:
            sys.exit(0)  # Ros shutdown
        # Reset the watcher period to its default.
        unused_watcher_period_response = gateway_watcher_period_service(gateway_srvs.SetWatcherPeriodRequest(-1.0))
        # Set up services 
        rospy.loginfo("Pairing Master : initialising simple client invitation service [%s]" % remote_invite_service_name)
        self.relay_invitation_server = rospy.Service('~invite', rocon_app_manager_srvs.SimpleInvite, self.relayed_invitation)
        self.watchdog_subscriber = rospy.Subscriber('~watchdog', std_msgs.Bool, self.watchdog_flag_cb)
        
        if self.auto_invite:
            rospy.loginfo("Pairing Master : auto-invite mode, disabling the cleanup watchdog.")
            self.watchdog_flag = False
            self.relayed_invitation(rocon_app_manager_srvs.SimpleInviteRequest(False))

    def watchdog_flag_cb(self, incoming):
        '''
          Used to keep an eye on android connections. If for any reason the android app connects, and disappears
          without releasing control of the rapp manager, then this automatically does so after a timeout.
          
          Just toggle a flag here that is watched in the spin() thread.   
        '''
        self.watchdog_flag = incoming.data
        if self.watchdog_flag:
            rospy.loginfo("Pairing Master : enabling the cleanup watchdog.")
        else:
            rospy.loginfo("Pairing Master : disabling the cleanup watchdog.")
        
    def is_pairing_device_present(self, master):
        pubs, unused_subs, unused_xxx = master.getSystemState()
        android_app_name_publishers = [x for x in pubs if x[0] == '/pairing_master/android_app_name']
        if len(android_app_name_publishers) == 0:
            return False
        else:
            return True
        
    def spin(self):
        '''
          If the private master's robot app manager is currently being remote controlled by us (start_app and stop_app
          is available), then it checks to make sure the android client is there. If not, it uninvites the private
          robot app manager so that it is free to be remote controlled by other sources.
          
          To check that it is there, it looks to see if either the android remocon or android remocon app is
          publishing to the /pairing/android_app_name topic.
        '''
        master = rosgraph.Master(rospy.get_name())
        flagged_for_release_count = 0
        while not rospy.is_shutdown():
            if not self.watchdog_flag:
                rospy.rostime.wallsleep(1.0)
            else:
                if flagged_for_release_count == 0:
                    rospy.rostime.wallsleep(1.0)
                else:
                    rospy.rostime.wallsleep(0.25)
                try:
                    # If not found, exceptions get thrown.
                    result = master.lookupService('/' + self.remote_gateway_name + '/start_app')
                    # Found, so check that an android client is connected.
                    if not self.is_pairing_device_present(master):
                        # Don't automatically disengage as sometimes the start_app handle will appear before the android
                        # client's handle. Put it under observation
                        flagged_for_release_count += 1
                        # This gives it 3-4s to release control, don't set it too low, because switching between a running app and
                        # back to the app list can often take 2-3 seconds before the watchdog topic is re-established.
                        if flagged_for_release_count == 20:
                            # Android client disappeared, probably crashed, so release control (uninvite)
                            rospy.loginfo("Pairing Master : android client disappeared, releasing remote control.")
                            remote_response = self.remote_invite_service(rocon_app_manager_srvs.InviteRequest(
                                                                     remote_target_name=self.local_gateway_name,
                                                                     application_namespace='',
                                                                     cancel=True))
                            flagged_for_release_count = 0
                    else:
                        flagged_for_release_count = 0
                except rospy.service.ServiceException:
                    pass # Was in the middle of uninviting when ros shutdown
                except rosgraph.masterapi.Error:
                    pass
                except rosgraph.masterapi.Failure:
                    pass
    
    def relayed_invitation(self, req):
        '''
          Provides a relayed invitation from a client (e.g. android remocon).
          This relay fills in the gateway name to pass on to the app manager,
          which makes life much simpler for the clients.
        '''
        if self.auto_invite and req.cancel:
            # Don't cancel, just ignore it and send back a true response (not broken).
            return rocon_app_manager_srvs.SimpleInviteResponse(True)
        try:
            rospy.loginfo("Pairing Master : inviting the private master's application manager.")
            remote_response = self.remote_invite_service(rocon_app_manager_srvs.InviteRequest(
                                                         remote_target_name=self.local_gateway_name,
                                                         application_namespace='',
                                                         cancel=req.cancel))
        except rospy.service.ServiceException:  # service call failed
            console.logerror("Pairing Master: remote invitation failed to connect.")
        except rospy.exceptions.ROSInterruptException:  # shutdown exception
            sys.exit(0)
        return rocon_app_manager_srvs.SimpleInviteResponse(remote_response.result)
    
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
    invitation_handler = InvitationHandler(local_gateway_name, remote_gateway_name, auto_invite)
    invitation_handler.spin()
