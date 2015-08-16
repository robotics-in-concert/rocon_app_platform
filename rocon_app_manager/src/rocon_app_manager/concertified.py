#
# License: BSD
#   https://raw.github.com/robotics-in-concertified/rocon_app_platform/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: concertified
   :platform: Unix
   :synopsis: A rapp manager class for connection with subserviant concerts.


A simpler app manager to use for connections to concerts than that used for
rocon indigo.

----

"""

##############################################################################
# Imports
##############################################################################

import gateway_msgs.msg as gateway_msgs
import rocon_python_comms
import rospy
#import std_msgs.msg as std_msgs

# local imports
from .standalone import Standalone
from .ros_parameters import ConcertParameters

##############################################################################
# Rapp Manager
##############################################################################


class Concertified(Standalone):
    '''
    A rapp manager for use with subserviant concerts.

    **Features**

    * **application namespace** - communicates with a gateway to set a unique application namespace in /concertified/clients/_name_
    * **firing the gateway** - pings the gateway whenever a rapp is started to create low latency flips

    **Publishers**

    * **~force_update** (*rapp_manager_msgs.IncompatibleRappList*) - rapps filtered from the startup list for various reasons [latched]

    .. todo:: if we want low latency flipping, call the gateway watcher set period service with low period after starting a rapp
    '''

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        super(Concertified, self).__init__()
        gateway_info_service = rocon_python_comms.SubscriberProxy('~gateway_info', gateway_msgs.GatewayInfo)
        rate = rospy.Rate(1)  # 10hz
        while not rospy.is_shutdown():
            gateway_info = gateway_info_service(timeout=rospy.Duration(0.3))
            if gateway_info:
                self.parameters.application_namespace = gateway_info.name.lower().replace(' ', '_')
                rospy.loginfo("Rapp Manager : setting application namespace to '%s'" % self.parameters.application_namespace)
                break
            try:
                rate.sleep()
                rospy.logwarn("Rapp Manager : unable to find the local gateway.")
            except rospy.exceptions.ROSInterruptException:
                rospy.loginfo("Rapp Manager : breaking out of gateway search loop [most likely just ros shutting down]")
        self.concert_parameters = ConcertParameters()

    ##########################################################################
    # Execute
    ##########################################################################

    def spin(self):
        """
        A default spinner. Child classes will overload this with their own custom spinners.
        """
        rospy.spin()
