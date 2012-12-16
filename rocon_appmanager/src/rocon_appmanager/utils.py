#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_appmanager/LICENSE
#
##############################################################################
# Imports
##############################################################################

import roslib.names
from .exceptions import NotFoundException


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
