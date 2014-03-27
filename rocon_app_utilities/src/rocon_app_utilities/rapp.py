#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################

from __future__ import division, print_function
import yaml
from .exceptions import *
import rocon_uri

#################################################################################
# Local Method
#################################################################################


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


def load_yaml_from_file(filename):
    '''
      Load rapp specs yaml from the given file

      :param filename: absolute path to the rapp definition
      :type filename: str

      :returns: dict of loaded rapp
      :rtype: dict

      :raises: InvalidRappFieldException: Rapp includes invalid filed
    '''
    RAPP_ATTRIBUTES = ['display', 'description', 'icon', 'public_interface', 'public_parameters', 'compatibility', 'launch', 'parent_name', 'pairing_clients', 'required_capability']

    with open(filename, 'r') as f:
        app_data = yaml.load(f.read())

        for d in app_data:
            if d not in RAPP_ATTRIBUTES:
                raise InvalidRappException('Invalid Field : [' + str(d) + '] Valid Fields : [' + str(RAPP_ATTRIBUTES) + ']')

    return app_data


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


def validate_rapp_field(data):
    '''
        Validate each field. E.g) check rocon uri. Check the linked file exist
    '''
    raise NotImplementedError()
    #  TODO
    pass


class Rapp(object):
    '''
        Rocon(or Robot) App definition.
    '''
    __slots__ = ['resource_name', 'data', 'type', 'is_implementation', 'is_ancestor', 'ancestor_name', 'parent_name']

    def __init__(self, name, filename=None):
        self.resource_name = name
        self.data = {}
        self.type = None
        self.ancestor_name = None
        self.is_implementation = False
        self.is_ancestor = False

        if filename:
            self.load_from_file(filename)
            #validate_rapp_field(self.dat)
            self.classify()

    def __str__(self):
        return str(self.resource_name + ' - ' + self.type)

    def is_compatible(self, uri):
        '''
          it compares its compatibility with given uri. and returns true if it is compatible.
          If it is virtual rapp which does not have compatibility field, return True always

          :param uri: rocon_uri
          :type uri: rocon_uri.RoconURI

          :returns: true if compatible
          :rtype: bool
        '''
        my_uri = self.data['compatibility'] if 'compatibility' in self.data else None

        if my_uri:
            return rocon_uri.is_compatible(my_uri, uri)
        else:
            return True

    def classify(self):
        '''
          classifies itself. It sets the following member variables.

          parent_name        : It has parent if it is child
          is_implementation  : Is it implementation?
          is_ancestor        : Is it Ancestor?
          type               : str(Virtual Ancestor, Implementation Ancestor or Implementation Child)
        '''
        self.parent_name = self.data['parent_name'] if 'parent_name' in self.data else None
        self.is_implementation, self.is_ancestor, self.type = classify_rapp_type(self.data)

    def load_from_file(self, filename):
        '''
          loads rapp data from file. and classifies itself.

          :param filename: absolute path to rapp definition
          :type filename: str
        '''
        self.data = load_yaml_from_file(filename)
        self.classify()

    def inherit(self, rapp):
        '''
          Inherits missing information from the given rapp

          :param rapp: rapp to inherit
          :type rapp: rocon_app_utilities.Rapp
        '''
        INHERITABLE_ATTRIBUTES = ['display', 'description', 'icon', 'public_interface', 'public_parameters', 'parent_name']

        # Once it inherits, it removes parent_specification field. If it is inherits from another child, it obtains parent_specification anyway
        del self.data['parent_name']

        for attribute in INHERITABLE_ATTRIBUTES:
            if not attribute in self.data and attribute in rapp.data:
                self.data[attribute] = rapp.data[attribute]

        self.classify()


class RappValidation(Rapp):
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
    _not_allowed = ['compatibility', 'launch', 'parent_name', 'pairing_clients', 'required_capability']


class ImplementationAncestorRapp(RappValidation):
    _required = ['display', 'description', 'compatibility', 'launch']
    _optional = ['icon', 'pairing_clients', 'required_capability', 'public_interface', 'public_parameters']
    _not_allowed = ['parent_name']


class ImplementationChildRapp(RappValidation):
    _required = ['compatibility', 'launch', 'parent_name']
    _optional = ['icon', 'pairing_clients', 'required_capability']
    _not_allowed = ['public_interface', 'public_parameters']
