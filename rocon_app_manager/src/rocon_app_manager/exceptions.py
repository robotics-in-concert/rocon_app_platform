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


class InvalidRappException(Exception):
    '''
      Raised if the app definition is invalid.
    '''


class MissingCapabilitiesException(Exception):
    '''
      Raised if one or more required capabilities are missing.
    '''
    def __init__(self, missing_caps):
        self.missing_caps = missing_caps
