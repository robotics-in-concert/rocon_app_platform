#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################

from .exceptions import *

def classify_rapp_type(data):
    '''
      Classify the current rapp among VirtualAncestor, ImplementationAnacestor, ImplementationChild

      :param data: rapp yaml data
      :type data: dict

      :returns: its classification is implementation? is ancestor? and its type
      :rtypes: bool, bool, str

      :raises: InvalidRappException: It is virtual child rapp
      :raises: InvalidRappFieldException: Rapp contains invalid field
    '''
    is_impl = _is_implementation_rapp(data)
    is_ance = _is_ancestor_rapp(data)

    impl = 'Implementation' if is_impl else 'Virtual'
    ance = 'Ancestor' if is_ance else 'Child'
    try:
        if is_impl and is_ance:  # Implementation Ancestor
            ImplementationAncestorRapp.is_valid(data)
        elif is_impl and not is_ance:  # Implementation Child
            ImplementationChildRapp.is_valid(data)
        elif not is_impl and is_ance:  # Virtual Ancestor
            VirtualAncestorRapp.is_valid(data)
        else:                         # Virtual Child
            raise InvalidRappException('Virtual Child rapp. Invalid!')
    except InvalidRappFieldException as ife:
        raise ife

    t = str(impl + ' ' + ance)
    return is_impl, is_ance, t


def _is_implementation_rapp(data):
    '''
      It is implementation if it contains compatibility and launch attributes

      :param data: dict of rapp definition
      :type data: dict

      :returns: whether it is implementation or not
      :rtype: bool
    '''
    IMPLEMETATION_VALIDATION_LIST = ['launch', 'compatibility']

    r = set(IMPLEMETATION_VALIDATION_LIST)
    m = set(data.keys())

    return r.issubset(m)


def _is_ancestor_rapp(data):
    '''
      It is ancestor rapp if it does not have parent_name attribute

      :param data: dict of rapp definition
      :type data: dict

      :returns: whether it is ancestor or not
      :rtype: bool
    '''
    CHILD_VALIDATION_LIST = ['parent_name']

    r = set(CHILD_VALIDATION_LIST)
    m = set(data.keys())
    return (not r.issubset(m))


class RappValidation(object):
    _required = []
    _optional = []
    _not_allowed = []

    @classmethod
    def is_valid(cls, data):
        '''
            Rapp Validation. If it has all requirements and does not include any not_allowed attributes, it is valid rapp

            :param data: rapp specification
            :type data: dict

            :returns: whether it is valid spec
            :rtype: bool

            :raises: InvalidRappFieldException: It includes not allowed field or misses required field
        '''
        missing_required = cls._difference(cls._required, data.keys())
        included_not_allowed = cls._intersection(cls._not_allowed, data.keys())

        if len(missing_required) > 0 or len(included_not_allowed) > 0:
            raise InvalidRappFieldException(cls, missing_required, included_not_allowed)

        return True

    @classmethod
    def _intersection(cls, attributes, data):
        intersection = set(attributes).intersection(set(data))
        return list(intersection)

    @classmethod
    def _difference(cls, attributes, data):
        diff = set(attributes).difference(set(data))
        return list(diff)


class VirtualAncestorRapp(RappValidation):
    _required = ['display', 'description']
    _optional = ['icon', 'public_interface', 'public_parameters']
    _not_allowed = ['compatibility', 'launch', 'parent_name', 'pairing_clients', 'required_capabilities']


class ImplementationAncestorRapp(RappValidation):
    _required = ['display', 'description', 'compatibility', 'launch']
    _optional = ['icon', 'pairing_clients', 'required_capabilities', 'public_interface', 'public_parameters']
    _not_allowed = ['parent_name']


class ImplementationChildRapp(RappValidation):
    _required = ['compatibility', 'launch', 'parent_name']
    _optional = ['icon', 'pairing_clients', 'required_capabilities']
    _not_allowed = ['public_interface', 'public_parameters']

