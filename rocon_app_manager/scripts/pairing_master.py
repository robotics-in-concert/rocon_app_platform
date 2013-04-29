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

def platform_info(namespace):
    '''
      Calls the application manager's platform info service to get details
      about the robot type that the android needs before connecting.
      
      @return namespace of the app manager (matches the remote gateway name)
      @rtype string or None : string does not yet prefix a leading '/'
    '''
    platform_info_service_name = '/' + namespace + '/platform_info'
    platform_info_service = rospy.ServiceProxy(platform_info_service_name, rocon_app_manager_srvs.GetPlatformInfo)
    platform_info = None
    resp = None
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(platform_info_service_name, timeout=0.3)
        except rospy.exceptions.ROSException:  # timeout exceeded
            continue
        except (rospy.exceptions.ROSInterruptException, rospy.service.ServiceException):  # shutdown exception
            sys.exit(0)
        try:
            resp = None
            resp = platform_info_service()
        except rospy.service.ServiceException:  # shutdown exception
            sys.exit(0)
        if resp is not None:
            break
    if resp is None:  # yes, can still be none...after shutdown
        sys.exit(0)
    return (resp.platform_info.platform, resp.platform_info.system, resp.platform_info.robot, resp.platform_info.name)

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
    #invite_service = rospy.Service('invite', rapp_manager_srvs.GetPlatformInfo, self._process_platform_info)
    rospy.loginfo("Pairing Master : local gateway name [%s]" % local_gateway_name)
    remote_gateway_name = remote_gateway_name()
    rospy.loginfo("Pairing Master : remote gateway name [%s]" % remote_gateway_name)
    # this is how the android app chooser can find the namespace for the start_app etc. handles
    rospy.set_param('/robot/name', remote_gateway_name)
    unused_platform, unused_system, robot, unused_name = platform_info(remote_gateway_name)
    rospy.set_param('/robot/type', robot)
    invite_service_name = '/' + remote_gateway_name + '/invite'
    invite_service = rospy.ServiceProxy(invite_service_name, rocon_app_manager_srvs.Invite)
    try:
        rospy.loginfo("Pairing Master : waiting for invite service [%s]" % invite_service_name)
        rospy.wait_for_service(invite_service_name)
        if auto_invite:
            rospy.loginfo("Pairing Master : automatically taking control (inviting) the application manager.")
            invite_service(rocon_app_manager_srvs.InviteRequest(remote_target_name=local_gateway_name,
                                                         application_namespace='',
                                                         cancel=False))
    except rospy.service.ServiceException:  # service call failed
        console.logerror("Pairing Master: invite service call failed.")
        sys.exit(1)
    except rospy.exceptions.ROSInterruptException:  # shutdown exception
        sys.exit(0)

    rospy.spin()
