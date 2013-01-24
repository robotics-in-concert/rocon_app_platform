#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_app_manager/LICENSE
#
##############################################################################
# Imports
##############################################################################

##############################################################################
# Exceptions
##############################################################################


class AppException(Exception):
    """
     App Exception
    """
    pass


class NotFoundException(AppException):
    """
      Resource Not Found Exception
    """
    pass


class IncompatibleAppException(AppException):
    """
      App not compatible with this platform.
    """
    pass


class InvalidPlatformTupleException(Exception):
    """
      Platform tuple invalid (must be platform.system.robot
      using strings from concert_msgs.Constants
    """
    pass


class InvalidRappException(Exception):
    '''
      Raised if the app definition is invalid.
    '''
