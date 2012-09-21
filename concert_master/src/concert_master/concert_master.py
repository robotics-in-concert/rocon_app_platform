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

import roslib; roslib.load_manifest('concert_master')
import rospy
from gateway_comms.srv import *

class ConcertMaster(object):
  publishers = {}
  subscribers = {}

  gateway_srv = None
  gateway_srv_name = '/gateway/request'

  gatewayinfo_srv_name = '/gateway/info'
  
  concertmaster_key = "rocon:concertmasterlist"
  
  def __init__(self):
    self.gateway_srv = rospy.ServiceProxy(self.gateway_srv_name,PublicHandler)
    self.gateway_srv.wait_for_service()
    self.gateway_info_srv = rospy.ServiceProxy(self.gatewayinfo_srv_name,GatewayInfo)
    self.cm_name = "concertmaster2"


  def spin(self):
    resp = self.gateway_info_srv()
    command = "post"
    self.fullname = self.cm_name + "," + resp.gateway_name
    msg = ["addmember",self.concertmaster_key,self.fullname]
    print "Advertising concertmaster = " + str(msg)
    resp = self.gateway_srv(command,msg)
    print resp

    rospy.spin()

    msg = ["removemember",self.concertmaster_key,self.fullname]
    print "Stop advertising concertmaster = " + str(msg)
    resp = self.gateway_srv(command,msg)
    print resp
