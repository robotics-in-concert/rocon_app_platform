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
from .rapp_validation import classify_rapp_type 
from .rapp_loader import load_rapp_yaml_from_file, load_rapp_specs_from_file
import rospkg

#################################################################################
# Rapp Class 
#################################################################################


class Rapp(object):
    '''
        Rocon(or Robot) App definition.
    '''
    __slots__ = ['resource_name', 'yaml_data', 'raw_data', 'data', 'type', 'is_implementation', 'is_ancestor', 'ancestor_name', 'parent_name', 'rospack', 'package', 'filename']

    def __init__(self, name, rospack=rospkg.RosPack(), filename=None):
        self.resource_name = name
        self.yaml_data = {}
        self.raw_data = {}
        self.data = {}
        self.type = None
        self.ancestor_name = None
        self.is_implementation = False
        self.is_ancestor = False
        self.rospack = rospack
        self.filename = None

        if filename:
            self.load_rapp_yaml_from_file(filename)

        #self.package = None

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
        my_uri = self.raw_data['compatibility'] if 'compatibility' in self.raw_data else None

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
        self.parent_name = self.raw_data['parent_name'] if 'parent_name' in self.raw_data else None
        self.is_implementation, self.is_ancestor, self.type = classify_rapp_type(self.raw_data)

    def load_rapp_yaml_from_file(self, filename):
        '''
          loads rapp data from file. and classifies itself.

          :param filename: absolute path to rapp definition
          :type filename: str
        '''

        try:
            self.yaml_data, self.raw_data = load_rapp_yaml_from_file(filename)
            self.filename = filename
            self.classify()
        except RappResourceNotExistException as e:
            raise InvalidRappException(str(self.resource_name) + ' : ' + str(e))

    def load_rapp_specs_from_file(self):
        '''
           Specification consists of resource which is file pointer. This function loads those files in memeory
        '''
        self.data = load_rapp_specs_from_file(self)

    def inherit(self, rapp):
        '''
          Inherits missing information from the given rapp

          :param rapp: rapp to inherit
          :type rapp: rocon_app_utilities.Rapp
        '''
        INHERITABLE_ATTRIBUTES = ['display', 'description', 'icon', 'public_interface', 'public_parameters', 'parent_name']

        # Once it inherits, it removes parent_specification field. If it is inherits from another child, it obtains parent_specification anyway
        del self.raw_data['parent_name']

        for attribute in INHERITABLE_ATTRIBUTES:
            if not attribute in self.raw_data and attribute in rapp.raw_data:
                self.raw_data[attribute] = rapp.raw_data[attribute]

        self.classify()
