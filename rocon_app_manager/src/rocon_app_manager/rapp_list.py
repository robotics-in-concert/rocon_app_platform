#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_app_manager/LICENSE
#
##############################################################################
# Overview
##############################################################################
'''
 Rapp lists store the apps that are being managed by the rapp manager. These
 can be preinstalled, installed or available apps from an rapp store.

 Note: very much a work in progress (not currently used). Pull code from our
 old willow app manager:

 https://github.com/robotics-in-concert/rocon_linux_app_platform/blob/master/willow_app_manager/src/willow_app_manager/app_list.py
'''
##############################################################################
# Imports
##############################################################################

import os
import rospy
import yaml
from .rapp import Rapp

##############################################################################
# Class
##############################################################################


class RappListFile(object):
    """
    Models data stored in a .rapps file.  These files are used to
    track apps available for the app manager. The apps file is
    just a single key 'apps' containing a list of 'xxx/yyy' strings
    where 'xxx' represents the package name and 'yyy' is the app name.
    """

    def __init__(self, filename):
        '''
          Just configures the container with basic parameters.

          @param filename : file path to the .rapps file.
          @type str
        '''
        if os.path.isfile(filename):
            self.filename = filename
            self.available_apps = []
            self._file_mtime = None
            self.update()
        else:
            raise IOError("rapp list file not found [%s]" % filename)

    def _load(self):
        available_apps = []
        rospy.loginfo("App Manager : loading apps file [%s]" % self.filename)
        with open(self.filename) as f:
            apps_yaml = yaml.load(f)
            if not 'apps' in apps_yaml:
                rospy.logerr("App Manager : apps file [%s] is missing required key [%s]" % (self.filename, 'apps'))
            for app_resource_name in apps_yaml['apps']:
                app = Rapp(app_resource_name)
                available_apps.append(app)
        self.available_apps = available_apps

    def update(self):
        """
        Update app list
        """
        try:
            s = os.stat(self.filename)
            if s.st_mtime != self._file_mtime:
                self._load()
                self._file_mtime = s.st_mtime
        except OSError as e:
            # Should only get here if someone deleted our file
            rospy.logerr("App Manager : tried to read a file that no longer exists [%s][%s]" % (self.filename, str(e)))


class RappList(object):

    def __init__(self, applist_directories):
        self._applist_directories = applist_directories
        self.installed_files = {}
        self.invalid_installed_files = []
        self.app_list = []

        self._applist_directory_mtime = None
        self.update()

    def add_directory(self, directory):
        self._applist_directories.append(directory)
