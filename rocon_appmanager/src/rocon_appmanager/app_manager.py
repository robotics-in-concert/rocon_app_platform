#!/usr/bin/env python       
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot, Daniel Stonier, Jihoon Lee
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Yujin Robot nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import os
from .app import App
"""
  AppManager - Jihoon Lee(jihoonl@yujinrobot.com)

  Feature:

  Todo:
    Configuration
      - app_list_directory by yaml
      - app_store url      by yaml

    Concert Master Relation
      - When a new concert master comes in network, it flips out publisher that informs platform-info, and service to listen an invitation.
      - When it receives a invitation from concert master, it validates the inviter with it's white/black list, then closes channels to other concert masters.
      - Have install/uninstall/start/stop an app services
  
    App Management
      - Load installed apps from app_list directory. 
      - have publisher that send out installed app list, available app list, and some more info. maybe latched=true one.
      - 'apt-get' app from app store

"""

class AppManager(object):

  param = {}
  apps = {}
  app_list = None
  DEFAULT_APP_LIST_DIRECTORY = '/opt/ros/'

  def __init__(self):

    # load configuration from rosparam
    rospy.loginfo("Parsing Parameters")
    self.parseParams()

    # It sets up an app directory and load installed app list from directory
    rospy.loginfo("Loading app lists")
    self.getInstalledApplist()
    rospy.loginfo("Done")


  def parseParams(self):
    param = {}
    param['app_from_source_directory'] = rospy.get_param('~app_from_source_directory',self.DEFAULT_APP_LIST_DIRECTORY)
    param['app_store_url'] = rospy.get_param('~app_store_url','')
    param['platform_info'] = rospy.get_param('~platform_info','')
    param['white_list'] = rospy.get_param('~whitelist','')
    param['black_list'] = rospy.get_param('~black_list','')

    self.param = param


  # it sets up an app directory and load installed app from given directory
  def getInstalledApplist(self):
    apps = {}
    
    # Getting apps from source
    directory = self.param['app_from_source_directory']
    apps['from_source'] = self.load(directory,'.app')
    apps['from_source'] = [App(a[0],a[1]) for a in apps['from_source']]

    # Getting apps in store
    print str(apps['from_source'])


    
  # It searchs *.app in directories
  def load(self,directory,typ):
    applist = []
    for dpath,_ ,files in os.walk(directory):
      apps = [f for f in files if f.endswith(typ)]
      apps_with_path = [dpath + '/' +  a for a in apps]
      apps_name = [a[0:len(a)-len(typ)] for a in apps]

      applist += list(zip(apps_name,apps_with_path))

    return applist        


    
  def spin(self):
    rospy.spin()


