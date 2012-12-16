#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_platform/master/rocon_appmanager/LICENSE
#
##############################################################################
# Overview
##############################################################################
'''
 App lists store the apps that are being managed by the app manager. These
 can be preinstalled, installed or available apps from an app store.

 Note: very much a work in progress (not currently used). Pull code from our
 old willow app manager:

 https://github.com/robotics-in-concert/rocon_linux_app_platform/blob/master/willow_app_manager/src/willow_app_manager/app_list.py
'''
##############################################################################
# Imports
##############################################################################

##############################################################################
# Functions
##############################################################################


def get_default_applist_directory():
    """
    Default directory where applist configuration is stored.
    """
    return "/etc/robot/apps"


##############################################################################
# Class
##############################################################################

class AppList(object):

    def __init__(self, applist_directories):
        self._applist_directories = applist_directories
        self.installed_files = {}
        self.invalid_installed_files = []
        self.app_list = []

        self._applist_directory_mtime = None
        self.update()

    def add_directory(self, directory):
        self._applist_directories.append(directory)
