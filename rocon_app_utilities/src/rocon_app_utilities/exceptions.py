#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################


class RappException(Exception):
    """
        Rapp Exception
    """
    pass


class InvalidRappException(RappException):
    '''
        Invalid format of rapp
    '''
    pass


class ParentRappNotFoundException(RappException):
    '''
        Parent Not Found Exception
    '''
    def __init__(self, resource_name, parent_name):
        self.resource_name = resource_name
        self.parent_name = parent_name


class RappInvalidChainException(RappException):
    '''
        If the rapp chain is invalid.
    '''
    pass


class RappAncestorConflictException(RappException):
    '''
        If indexer found two implementation with the same ancestor
    '''
    pass


class RappCyclicChainException(RappException):
    '''
        If the rapp includes cyclic chain. e.g child -> parent -> child -> ....
    '''
    def __init__(self, stack):
        self.stack = stack

    def __strc__(self):
        return str(self.stack)


class RappNotExistException(RappException):
    '''
        When Rapp does not exist
    '''
    pass


class InvalidRappFieldException(RappException):
    '''
        It does not satisfy required or not allowed field
    '''
    def __init__(self, cls, invalid_required, invalid_not_allowed):
        self.cls = cls
        self.invalid_required = invalid_required
        self.invalid_not_allowed = invalid_not_allowed

    def __str__(self):
        return str('\n\t' + str(self.cls) + '\n\tMissing Requirements - ' + str(self.invalid_required) + '\n\tInvalid Not Allowed - ' + str(self.invalid_not_allowed))


class RappResourceNotExistException(RappException):
    '''
      Rapp Attribute Resource does not exist...
    '''
    pass


class RappMalformedException(RappException):
    '''
      If rapp contains missing key...
    '''


class UnsupportedPlatformException(Exception):
    '''
      If running on a platform not supported by rosdep.
    '''
    pass


class NonInstallableRappException(Exception):
    '''
      If Rapp cannot be installed.
    '''
    pass

class XmlParseException(Exception):
    '''
      Error with the XML syntax (e.g. invalid attribute/value combinations)
    '''
    pass
