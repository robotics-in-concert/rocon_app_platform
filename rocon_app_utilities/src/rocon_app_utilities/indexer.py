#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################

from __future__ import division, print_function 
import copy

#from .rapp import Rapp, MetaRapp
import rocon_python_utils
import rocon_uri

from .exceptions import *
from .rapp import Rapp
from rocon_console import console

class RappIndexer(object):

    __slots__ = ['raw_data_path', 'raw_data', 'invalid_data', 'package_whitelist', 'package_blacklist']

    def __init__(self, raw_data=None, package_whitelist=None, package_blacklist=[]):
        self.raw_data_path = {}
        self.raw_data = {}
        self.package_whitelist = package_whitelist
        self.package_blacklist = package_blacklist 

        if raw_data:
            self.raw_data = raw_data
        else:
            self.update_index()


    def update_index(self, package_whitelist=None, package_blacklist=[]):
        '''
          Crawls rocon apps from ROS_PACKAGE_PATH and generates raw_data dictionary.  

          :param package_whitelist: list of target package list
          :type package_whitelist: [str]
          :param package_blacklist: list of blacklisted package
          :type package_blacklist: [str]
        '''
        self.raw_data_path, _invalid_path = rocon_python_utils.ros.resource_index_from_package_exports('rocon_app', None, package_whitelist, package_blacklist)
        raw_data = {}
        invalid_data = {}

        for resource_name, path in self.raw_data_path.items():
            try:
                r = Rapp(resource_name)
                r.load_from_file(path)
                r.classify()
                raw_data[resource_name] = r
            except InvalidRappFieldException as irfe:
                invalid_data[resource_name] = str(irfe)
#console.warning('  [' + resource_name + '] has not been added : ' + str(irfe))
            except InvalidRappException as ire:
                invalid_data[resource_name] = str(ire)
#console.warning('  [' + resource_name + '] has not been added : ' + str(ire))
        self.raw_data = raw_data
        self.invalid_data = invalid_data
        self.package_whitelist = package_whitelist
        self.package_blacklist = package_blacklist

    def get_package_whitelist_blacklist(self):
        return self.package_whitelist, self.package_blacklist

    def get_raw_rapp(self, rapp_name):
        '''
          returns rapp instance of given name

          :param rapp_name: rapp name
          :type rapp_name: str

          :returns: rapp
          :rtype: rocon_app_utilities.Rapp

          :raises: RappNotExistException: the given rapp name does not exist
        '''
        if not rapp_name in self.raw_data:
            raise RappNotExistException(str(rapp_name) + ' does not exist')

        return self.raw_data[rapp_name]

    def get_rapp(self, rapp_name):
        '''
          returns complete rapp instance which includes inherited attributes from its parent

          :param rapp name: Rapp name
          :type rapp_name: str

          :returns: rapp instance
          :rtype: rocon_app_utilities.rapp.Rapp

          :raises: RappNotExistException: the given rapp name does not exist
        '''
        if not rapp_name in self.raw_data:
            raise RappNotExistException(str(rapp_name) + ' does not exist')

        rapp = self._resolve(rapp_name)

        return rapp

    def get_compatible_rapps(self, uri=rocon_uri.default_uri_string):
        '''
          returns all rapps which are compatible with given URI

          :param uri: Rocon URI
          :type uri: str 
          
          :returns: a list of compatible rapps, a list of incompatible rapps
          :rtype: [rocon_app_utilities.Rapp], [rocon_app_utilities.Rapp]
        '''
        compatible_rapps = {resource_name: rapp for resource_name, rapp in self.raw_data.items() if rapp.is_implementation and rapp.is_compatible(uri)}
        incompatible_rapps = {resource_name: rapp for resource_name, rapp in self.raw_data.items() if rapp.is_implementation and not resource_name in compatible_rapps}

        #  TODO: Utilise invalid list later for better introspection
        resolved_compatible_rapplist, _ = self._resolve_rapplist(compatible_rapps)
        resolved_incompatible_rapplist, _ = self._resolve_rapplist(incompatible_rapps)

        return resolved_compatible_rapplist, resolved_incompatible_rapplist

    def _resolve_rapplist(self, rapps):
        '''
          resolve full spec of given dict of rapps 

          :param rapps: list of rapps
          :type dict

          :returns: resolved rapp list, invalid rapp list
          :rtypes: [], []
        '''
        resolved_list = [] 
        used_ancestors = {}
        invalid_list = []
        for resource_name, rapp in rapps.items():
            resolved_rapp = self.get_rapp(resource_name)
            ancestor_name = resolved_rapp.ancestor_name
            if ancestor_name in used_ancestors:
                invalid_list.append(resource_name)
            resolved_list.append(resolved_rapp)
            used_ancestors[ancestor_name] = resource_name 
        return resolved_list, invalid_list 

    def _resolve(self, rapp_name):
        '''
          resolve the rapp instance with its parent specification and return a runnable rapp

          :param rapp name: Rapp name
          :type rapp_name: str

          :returns: fully resolved rapp  
          :rtype: rocon_utilities.Rapp
        '''
        rapp = copy.deepcopy(self.raw_data[rapp_name]) # Not to currupt original data
        parent_name = rapp.parent_name
        stack = []
        stack.append(rapp.resource_name)
        rapp, ancestor_name = self._resolve_recursive(rapp, parent_name, stack)
        rapp.ancestor_name = ancestor_name

        return rapp

    def _resolve_recursive(self, rapp, parent_name, stack):
        '''
            Internal method of _resolve

            :raises: RappInvalidChainException: Rapp is implmentation child but does not have parent
            :raises: ParentRappNotFoundException: Its parent does not exist
        '''
        if rapp.is_ancestor and rapp.is_implementation:
            return rapp, stack.pop()

        if not parent_name:
            raise RappInvalidChainException('Invalid Rapp Chain from [' + str(rapp)+']')

        if not parent_name in self.raw_data:
            raise ParentRappNotFoundException(rapp.resource_name,parent_name)

        if parent_name in stack:
            raise RappCyclicChainException(stack)

        parent = self.raw_data[parent_name]
        rapp.inherit(parent)
        stack.append(parent.resource_name)

        return self._resolve_recursive(rapp, parent.parent_name, stack)

    def to_dot(self):
        '''
            returns the dot graph format. Not Implemented Yet.
        '''
        raise NotImplementedError()
        # TODO
        pass
