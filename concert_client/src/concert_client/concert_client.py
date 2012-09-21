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

import roslib;roslib.load_manifest('concert_client')

import rospy
import threading
from gateway_comms.srv import *

"""
  ConcertClient - Jihoon Lee(jihoonl@yujinrobot.com)

  Concert Master Relation
    - When a new concert master comes in network, it flips out publisher that informs platform-info, and service to listen an invitation.
    - When it receives a invitation from concert master, it validates the inviter with it's white/black list, then closes channels to other concert masters.
    - Have install/uninstall/start/stop an app services
"""

# It polls redis server to discover currently connected concertmaster 
# and notifis concert client
class ConcertMasterDiscovery(threading.Thread):
    params = {}
      
    def __init__(self,service,cmlist_key,processNewMaster):
        threading.Thread.__init__(self)
        self.service = service
        self.concertmasterlist_key = cmlist_key

        self.processNewMaster = processNewMaster
        self._stop = False
        self.start()

    def run(self):
        print "Concert Master Discovery has started"
        command = "post"
        msg = ["getmembers",self.concertmasterlist_key,""]

        while not rospy.is_shutdown() and not self._stop:
            resp = self.service(command,msg)
            print str(resp.concertmaster_list)
            rospy.sleep(3)

        print "Concert Master Discovery has stopped"

    def setStop(self):
        self._stop = True



class ConcertClient(object):
    subscribers = {}
    publishers = {}

    concertmasterlist = []
    concertmasterlist_key = "rocon:concertmasterlist"

    gateway_srv_name = "/gateway/request"
    gateway_srv = None

    def __init__(self,whitelist,blacklist):
        self.whitelist = whitelist
        self.blacklist = blacklist

        self.gateway_srv = rospy.ServiceProxy(self.gateway_srv_name,PublicHandler)
        self.masterdiscovery = ConcertMasterDiscovery(self.gateway_srv,self.concertmasterlist_key,self.processNewMaster)

 

    def spin(self):
        rospy.spin()

    def processNewMaster(self,disconvered_masterlist):
        print str(disconvered_masterlist)
      
      
