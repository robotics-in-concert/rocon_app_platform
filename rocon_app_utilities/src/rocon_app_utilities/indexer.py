#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################

from __future__ import division, print_function
import copy
import os
import tarfile
import tempfile

import rocon_python_utils
import rocon_uri
import rospkg

from .exceptions import *
from .rapp import Rapp

import logging
import sys
logger = logging.getLogger('indexer')
logger.addHandler(logging.StreamHandler(sys.stderr))
#logger.setLevel(logging.DEBUG)


class RappIndexer(object):

    __slots__ = ['raw_data_path', 'raw_data', 'invalid_data', 'package_whitelist', 'package_blacklist', 'rospack', 'packages_path', 'source']

    def __init__(self, raw_data=None, package_whitelist=None, package_blacklist=[], packages_path=None, source=None):
        self.packages_path = packages_path
        self.raw_data_path = {}
        self.raw_data = {}
        self.invalid_data = {}
        self.package_whitelist = package_whitelist
        self.package_blacklist = package_blacklist
        self.source = source
        self.rospack = rospkg.RosPack()

        if raw_data is not None:
            self.raw_data = raw_data
        else:
            self.update_index(package_whitelist, package_blacklist)

    def __str__(self):

        ret = '-------------------------------\n'
        for rapp_name, rapp in self.raw_data.items():
            ret += str(rapp_name) + '\n'
            for attr_name, attr_path in rapp.raw_data.items():
                ret += '    ' + str(attr_name) + ' : ' + str(attr_path) + '\n'

        ret += '--------------------------------'
        return ret

    def update_index(self, package_whitelist=None, package_blacklist=[]):
        '''
          Crawls rocon apps from ROS_PACKAGE_PATH and generates raw_data dictionary.

          :param package_whitelist: list of target package list
          :type package_whitelist: [str]
          :param package_blacklist: list of blacklisted package
          :type package_blacklist: [str]
        '''
        self.raw_data_path, _invalid_path = rocon_python_utils.ros.resource_index_from_package_exports('rocon_app', self.packages_path, package_whitelist, package_blacklist)
        raw_data = {}
        invalid_data = {}

        for resource_name, (path, catkin_package) in self.raw_data_path.items():
            try:
                r = Rapp(resource_name, self.rospack)
                r.load_rapp_yaml_from_file(path)
                r.package = catkin_package
                raw_data[resource_name] = r
            except InvalidRappFieldException as irfe:
                invalid_data[resource_name] = str(irfe)
            except InvalidRappException as ire:
                invalid_data[resource_name] = str(ire)
            except RappResourceNotExistException as e:
                invalid_data[resource_name] = str(e)
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
        rapp.load_rapp_specs_from_file()

        return rapp

    def get_compatible_rapps(self, uri=rocon_uri.default_uri_string, ancestor_share_check=False):
        '''
          returns all rapps which are compatible with given URI

          :param uri: Rocon URI
          :type uri: str

          :returns: a dict of compatible rapps, a dict of incompatible rapps, a dict of invalid rapps
          :rtype: {resource_name:rocon_app_utilities.Rapp}, {resource_name:rocon_app_utilities.Rapp}, {resource_name:str}
        '''
        compatible_rapps = {}
        incompatible_rapps = {}
        invalid_rapps = {}

        for resource_name, rapp in self.raw_data.items():
            if not rapp.is_implementation:
                continue
            try:
                if rapp.is_compatible(uri):
                    compatible_rapps[resource_name] = rapp
                else:
                    incompatible_rapps[resource_name] = rapp
            except rocon_uri.exceptions.RoconURIValueError as e:
                invalid_rapps[resource_name] = str(e)

        resolved_compatible_rapps, invalid_compatible = self._resolve_rapplist(compatible_rapps, ancestor_share_check)
        resolved_incompatible_rapps, invalid_incompatible = self._resolve_rapplist(incompatible_rapps, ancestor_share_check)
        invalid_rapps.update(invalid_compatible)
        invalid_rapps.update(invalid_incompatible)

        for resource_name, rapp in resolved_compatible_rapps.items():
            try:
                rapp.load_rapp_specs_from_file()
            except RappResourceNotExistException as e:
                invalid_rapps[resource_name] = str(e)
            except RappMalformedException as e:
                invalid_rapps[resource_name] = str(e)

        for resource_name in invalid_rapps:
            if resource_name in resolved_compatible_rapps:
                del resolved_compatible_rapps[resource_name]

        if hasattr(self, 'invalid_data'):
            invalid_rapps.update(self.invalid_data)

        return resolved_compatible_rapps, resolved_incompatible_rapps, invalid_rapps

    def _resolve_rapplist(self, rapps, ancestor_share_check):
        '''
          resolve full spec of given dict of rapps

          :param rapps: list of rapps
          :type dict

          :returns: resolved rapps, invalid rapps
          :rtypes: {}, {}
        '''
        resolved = {}
        used_ancestors = {}
        invalid = {}
        for resource_name, unused_rapp in rapps.items():
            try:
                resolved_rapp = self._resolve(resource_name)
                ancestor_name = resolved_rapp.ancestor_name
                if ancestor_share_check and ancestor_name in used_ancestors:
                    invalid[resource_name] = "Ancestor has already been taken by other rapp"
                else:
                    resolved[resource_name] = resolved_rapp
                used_ancestors[ancestor_name] = resource_name
            except ParentRappNotFoundException as e:
                invalid[resource_name] = str('Invalid parent_name [%s] in resource [%s]' % (str(e.parent_name), str(e.resource_name)))
            except RappInvalidChainException as e:
                invalid[resource_name] = str(e)
        return resolved, invalid

    def _resolve(self, rapp_name):
        '''
          resolve the rapp instance with its parent specification and return a runnable rapp

          :param rapp name: Rapp name
          :type rapp_name: str

          :returns: fully resolved rapp
          :rtype: rocon_app_utilities.Rapp
        '''
        rapp = copy.deepcopy(self.raw_data[rapp_name])  # Not to currupt original data
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
            raise RappInvalidChainException('Invalid Rapp Chain from [' + str(rapp) + ']')

        if not parent_name in self.raw_data:
            raise ParentRappNotFoundException(rapp.resource_name, parent_name)

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

    def merge(self, other_indexer):
        '''
          Updates this index with the rapps from the other_indexer.

          :param other_indexer: the other inder
          :type other_indexer: rocon_app_utilities.RappIndexer
        '''
        self.raw_data.update(other_indexer.raw_data)
        self.raw_data_path.update(other_indexer.raw_data_path)

        # Cleanup 'invalid' invalid data before merge
        self.invalid_data = {k: v for k, v in self.invalid_data.items() if k not in self.raw_data}
        self.invalid_data.update(other_indexer.invalid_data)

    def write_tarball(self, filename_prefix):
        '''
          Writes the index to a gzipped tarball.

          :param filename_prefix: the pathname of the archive with out the suffix '.index.tar.gz'
          :type filename_prefix: str
        '''

        RESOURCE_KEYS = ['icon', 'public_interface', 'public_parameters', 'launch']

        logger.debug("write_tarball() to '%s...'" % filename_prefix)
        added = set([])
        with tarfile.open('%s.index.tar.gz' % filename_prefix, 'w:gz') as tar:
            for rapp in self.raw_data.values():
                # add package.xml file
                rapp_package_filename = os.path.normpath(rapp.package.filename)
                if rapp_package_filename not in added:
                    logger.debug("write_tarball() add package.xml '%s" % rapp_package_filename)
                    tar.add(rapp_package_filename)
                    added.add(rapp_package_filename)
                # add .rapp file
                rapp_filename = os.path.normpath(rapp.filename)
                if rapp_filename not in added:
                    logger.debug("write_tarball() add .rapp file '%s" % rapp_filename)
                    tar.add(rapp_filename)
                    added.add(rapp_filename)

                    for value in [v for k, v in rapp.yaml_data.items() if k in RESOURCE_KEYS]:
                        logger.debug("write_index() value: %s" % str(value))

                        if value and os.path.exists(value):
                            normed_path = os.path.normpath(value)
                            logger.debug("write_index() add resource '%s" % str(normed_path))
                            tar.add(normed_path)
                            added.add(normed_path)
                        else:
                            logger.debug("write_index() path does not exist %s" % str(value))


def read_tarball(name=None, fileobj=None, package_whitelist=None, package_blacklist=[]):
    '''
      Reads an index from a gzipped tarball.

      :param name: the pathname of the archive
      :type name: str
      :param fileobj: alternative to a file object opened for name
      :type fileobj: file
      :param package_whitelist: list of target package list
      :type package_whitelist: [str]
      :param package_blacklist: list of blacklisted package
      :type package_blacklist: [str]

      :returns: the index
      :rtype: rocon_app_utilities.RappIndexer
    '''
    # TODO avoid unpacking
    logger.debug('read_tarball(name=%s, fileobj=%s)' % (name, fileobj))
    tempdir = tempfile.mkdtemp(suffix='_unpacked', prefix='rapp_index_')
    try:
        logger.debug("read_tarball() unpack to '%s'" % tempdir)
        with tarfile.open(name=name, fileobj=fileobj, mode='r:gz') as tar:
            tar.extractall(tempdir)
        index = RappIndexer(packages_path=tempdir, package_whitelist=package_whitelist, package_blacklist=package_blacklist)
    finally:
        #shutil.rmtree(tempdir)
        pass
    return index
