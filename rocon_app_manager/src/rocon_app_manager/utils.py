#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_std_msgs.msg as rocon_std_msgs
from .exceptions import InvalidPlatformTupleException

##############################################################################
# Classes
##############################################################################

# Should get these deprecated - the functionality is now in rocon_utilities.


class PlatformTuple(object):
    __slots__ = [
            'os',
            'version',
            'system',
            'platform'
        ]

    def __init__(self, platform_tuple):
        '''
          Converts a platform tuple string into a structure.
          @param platform_tuple os.version.system.robot
          @type string
        '''
        platform_tuple_list = platform_tuple.split('.')
        if len(platform_tuple_list) != 4:
            raise InvalidPlatformTupleException("len('%s') != 4" % platform_tuple)
        # should also validate against rapp_manager_msgs.PLatformInfo constant definitions
        self.os = platform_tuple_list[0]
        self.version = platform_tuple_list[1]
        self.system = platform_tuple_list[2]
        self.platform = platform_tuple_list[3]


def platform_tuple(os, version, system, platform):
    '''
      Return the platform tuple string identified by the four strings.
    '''
    return (os + '.' + version + '.' + system + '.' + platform)


def platform_compatible(first_platform_tuple, second_platform_tuple):
    '''
      Used to check platform compatibility between app manager
      and its apps.

      @param first_platform_tuple : os.version.system.platform
      @type string
      @param second_platform_tuple : os.version.system.platform
      @type string

      @return false or true depending on compatibility result
      @rtype bool
    '''
    try:
        platform_one = PlatformTuple(first_platform_tuple)
        platform_two = PlatformTuple(second_platform_tuple)
    except InvalidPlatformTupleException as e:
        rospy.logwarn("App Manager : invalid platform tuple [%s]" % str(e))
        return False
    if platform_one.os != rocon_std_msgs.PlatformInfo.OS_ANY and \
       platform_two.os != rocon_std_msgs.PlatformInfo.OS_ANY and \
       platform_one.os != platform_two.os:
        return False
    # Should check version here as well.
    if platform_one.system != platform_two.system:
        return False
    if platform_one.platform != rocon_std_msgs.PlatformInfo.SYSTEM_ANY and \
       platform_two.platform != rocon_std_msgs.PlatformInfo.SYSTEM_ANY and \
       platform_one.platform != platform_two.platform:
        return False
    return True
