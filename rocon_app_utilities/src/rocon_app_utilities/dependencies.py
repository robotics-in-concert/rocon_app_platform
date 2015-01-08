#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################

from collections import defaultdict
import os
import platform
import rospkg.os_detect 

from rosdep2 import RosdepLookup, create_default_installer_context, ResolutionError
from rosdep2.installers import RosdepInstaller
from rosdep2.sources_list import SourcesListLoader
from rosdep2.rospkg_loader import DEFAULT_VIEW_KEY

from rocon_python_utils.ros.resources import _get_package_index

from .exceptions import *


class RappDependencies(object):

    def __init__(self, rapp_name):
        self.rapp_name = rapp_name
        self.installable = []
        self.noninstallable = []
        self.installed = []

    def all_installed(self):
        '''
        Check if all dependencies are installed.
        '''
        return not self.installable and not self.noninstallable

    def any_not_installable(self):
        '''
        Check if any dependency is not installable.
        '''
        return len(self.noninstallable) > 0


class DependencyChecker(object):

    def __init__(self, indexer, ros_distro=None, os_name=None, os_codename=None):
        self.indexer = indexer

        if not os_name:
            os_detector = rospkg.os_detect.OsDetect()
            self.os_name = os_detector.get_name()
            self.os_codename = os_detector.get_codename()

        # FIXME: there should be a way to set this without using an environment variable
        if ros_distro:
            os.environ['ROS_DISTRO'] = ros_distro

        self.sources_loader = SourcesListLoader.create_default(verbose=False)
        self.lookup = RosdepLookup.create_from_rospkg(sources_loader=self.sources_loader)
        self.installer_context = create_default_installer_context(verbose=False)
        self.installer_keys = self.installer_context.get_os_installer_keys(self.os_name)

        self.default_key = self.installer_context.get_default_os_installer_key(self.os_name)
        self.installer = self.installer_context.get_installer(self.default_key)

        self.view = self.lookup.get_rosdep_view(DEFAULT_VIEW_KEY, verbose=False)

    def check_rapp_dependencies(self, rapp_names):
        '''
        Check the installation status of the dependencies for a given list of Rapps.

        :param rapp_names: A C{list} of ROCON URIs

        :returns: A C{dict} mapping rapp names to C{RappDependencies}.
        '''

        package_index = _get_package_index(None)
        pkgs = package_index.keys()

        deps = {}

        for rapp_name in rapp_names:
            deps[rapp_name] = RappDependencies(rapp_name)
            rapp = self.indexer.get_raw_rapp(rapp_name)
            for run_depend in rapp.package.run_depends:
                if run_depend.name not in pkgs:
                    # Dependency was not found in either the local tree or systemwide
                    try:
                        d = self.view.lookup(run_depend.name)
                        inst_key, rule = d.get_rule_for_platform(
                            self.os_name, self.os_codename, self.installer_keys, self.default_key
                        )  # os version used in rosdep is os_codename. refer to REP 111
                        deps[rapp_name].installable.extend(self.installer.resolve(rule))
                    except KeyError:
                        deps[rapp_name].noninstallable.append(run_depend.name)
                else:
                    deps[rapp_name].installed.append(run_depend.name)

        return deps

    def install_rapp_dependencies(self, rapp_names):
        '''
        Install the dependencies for a given list of Rapps.

        :param rapp_name: A C{list} of ROCON URIs
        :raises: NonInstallableRappException: Any of the ROCON URIs are not installable
        '''
        deps = self.check_rapp_dependencies(rapp_names)

        uninstallable_rapps = [k for k, v in deps.items() if v.noninstallable]
        if uninstallable_rapps:
            raise NonInstallableRappException(uninstallable_rapps)

        installable = [d for v in deps.values() for d in v.installable]
        pkg_deps = list(set(installable))

        rosdep_installer = RosdepInstaller(self.installer_context, self.lookup)
        rosdep_installer.install_resolved(self.default_key, pkg_deps)
