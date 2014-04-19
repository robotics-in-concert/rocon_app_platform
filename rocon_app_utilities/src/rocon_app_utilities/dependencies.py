#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/license/LICENSE
#
#################################################################################

from collections import defaultdict
import os
import platform

from rosdep2 import RosdepLookup, create_default_installer_context, ResolutionError
from rosdep2.installers import RosdepInstaller
from rosdep2.sources_list import SourcesListLoader
from rosdep2.rospkg_loader import DEFAULT_VIEW_KEY

from rocon_python_utils.ros.resources import _get_package_index

from .exceptions import *

class DependencyChecker(object):

    def __init__(self, indexer, ros_distro=None, os_name=None, os_id=None):
        self.indexer = indexer

        import os
        if not os_name:
            os_platform = platform.system().lower()
            if os_platform == 'linux':
                # Retrieve distribution name
                os_name, os_version, os_id = platform.linux_distribution()
                os_name = os_name.lower()
            else:
               # We only support Linux for now
               raise UnsupportedPlatformException(os_platform)

        self.os_name = os_name
        self.os_id = os_id

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

        :returns: A C{tuple} of three C{dicts}s.
        The fist dictionary maps Rapp names to installer rules of packages needed to install the rapp.
        The second dictionary maps the Rapp names to missing dependencies which are not installable.
        The third dictionary maps the Rapp name to its already installed dependencies.
        Note that a single rapp name might be listed in multiple if the return dictionaries.
        '''

        package_index = _get_package_index(None)
        pkgs = package_index.keys()

        installable_rapps = defaultdict(list)
        noninstallable_rapps = defaultdict(list)
        installed_rapps = defaultdict(list)

        for rapp_name in rapp_names:
            rapp = self.indexer.get_raw_rapp(rapp_name)
            for run_depend in rapp.package.run_depends:
                if run_depend.name not in pkgs:
                    # Dependency was not found in either the local tree or systemwide
                    try:
                        d = self.view.lookup(run_depend.name)
                        inst_key, rule = d.get_rule_for_platform(
                            self.os_name, self.os_id, self.installer_keys, self.default_key
                        )
                        installable_rapps[rapp_name].extend(self.installer.resolve(rule))
                    except KeyError:
                        noninstallable_rapps[rapp_name].append(run_depend.name)
                else:
                    installed_rapps[rapp_name].append(run_depend.name)

        return (installable_rapps, noninstallable_rapps, installed_rapps)

    def install_rapp_dependencies(self, rapp_names):
        '''
        Install the dependencies for a given list of Rapps.

        :param rapp_name: A C{list} of ROCON URIs
        :raises: NonInstallableRappException: Any of the ROCON URIs are not installable
        '''
        installable_rapps, uninstallable_rapps, _ = self.check_rapp_dependencies(rapp_names)

        if uninstallable_rapps:
            raise NonInstallableRappException(uninstallable_rapps.keys())

        rosdep_installer = RosdepInstaller(self.installer_context, self.lookup)

        pkg_deps = []
        for rapp_name, dependencies in installable_rapps.iteritems():
            pkg_deps.extend(dependencies)
        rosdep_installer.install_resolved(self.default_key, pkg_deps)
