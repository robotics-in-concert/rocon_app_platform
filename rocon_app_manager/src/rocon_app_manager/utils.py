#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_app_manager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import roslib
import roslib.names
import concert_msgs.msg as concert_msgs
from .exceptions import NotFoundException, InvalidPlatformTupleException

##############################################################################
# Classes
##############################################################################


class PlatformTuple(object):

    def __init__(self, platform_tuple):
        '''
          Converts a platform tuple string into a structure.
          @param platform_tuple platform.system.robot
          @type string
        '''
        platform_tuple_list = platform_tuple.split('.')
        if len(platform_tuple_list) != 3:
            raise InvalidPlatformTupleException("len('%s') != 3" % platform_tuple)
        # also validate against concert_msgs.Constants
        self.platform = platform_tuple_list[0]
        self.system = platform_tuple_list[1]
        self.robot = platform_tuple_list[2]


def findResource(resource):
    '''
      @return: filepath of resource.  Does not validate if filepath actually exists.
      @raise ValueError: if resource is not a valid resource name.
      @raise roslib.packages.InvalidROSPkgException: if package referred
        to in resource name cannot be found.
      @raise NotFoundException: if resource does not exist.
    '''

    p, a = roslib.names.package_resource_name(resource)

    if not p:
        raise ValueError("Resource is missing package name: %s" % (resource))
    matches = roslib.packages.find_resource(p, a)

    # TODO: convert ValueError to better type for better error messages
    if len(matches) == 1:
        return matches[0]
    elif not matches:
        raise NotFoundException("No resource [%s]" % (resource))
    else:
        print matches
        raise ValueError("Multiple resources named [%s]" % (resource))


def find_resource(resource):
    '''
      Ros style resource finder.

      @param resource is a ros resource (package/name)
      @type str
      @return full path to the resource
      @type str
      @raise NotFoundException: if resource does not exist.
    '''
    p, a = roslib.names.package_resource_name(resource)
    if not p:
        raise NotFoundException("resource is missing package name [%s]" % (resource))
    matches = roslib.packages.find_resource(p, a)
    if len(matches) == 1:
        return matches[0]
    elif not matches:
        raise NotFoundException("no resource [%s]" % (resource))
    else:
        #print matches
        raise NotFoundException("multiple resources found [%s]" % (resource))


def platform_tuple(platform, system, robot):
    '''
      Return the platform tuple string identified by the three strings.
    '''
    return (platform + '.' + system + '.' + robot)


def platform_compatible(first_platform_tuple, second_platform_tuple):
    '''
      Used to check platform compatibility between app manager
      and its apps.

      @param first_platform_tuple : platform.system.robot
      @type string
      @param second_platform_tuple : platform.system.robot
      @type string

      @return false or true depending on compatibility result
      @rtype bool
    '''
    try:
        platform_one = PlatformTuple(first_platform_tuple)
        platform_two = PlatformTuple(second_platform_tuple)
    except InvalidPlatformTupleException as e:
        rospy.logwarn("Rapp Manager : invalid platform tuple [%s]" % str(e))
        return False
    if platform_one.platform != concert_msgs.Constants.PLATFORM_ANY and \
       platform_two.platform != concert_msgs.Constants.PLATFORM_ANY and \
       platform_one.platform != platform_two.platform:
        return False
    if platform_one.system != platform_two.system:
        return False
    if platform_one.robot != concert_msgs.Constants.ROBOT_ANY and \
       platform_two.robot != concert_msgs.Constants.ROBOT_ANY and \
       platform_one.robot != platform_two.robot:
        return False
    return True
