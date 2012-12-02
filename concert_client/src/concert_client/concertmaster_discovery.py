#!/usr/bin/env python       
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_client/LICENSE 
#

import rospy
import threading
# It polls redis server to discover currently connected concertmaster 
# and notifis concert client
class ConcertMasterDiscovery(threading.Thread):
    params = {}
      
    def __init__(self,hub_client,key,processNewMaster,log,logerr):
        threading.Thread.__init__(self)
        self.hub_client = hub_client
        self.concertmasterlist_key = key
        self.processNewMaster = processNewMaster
        self.log = log
        self._stop = False
        self.masterlist = []

    def run(self):
        self.log("Concert Master Discovery has started")

        while not rospy.is_shutdown() and not self._stop:
            discovered_masterlist = self.getMasterList()
            self.processNewMaster(discovered_masterlist)
            rospy.sleep(3)

        self.log("Concert Master Discovery has stopped")

    def getMasterList(self):
        return self.hub_client.getValues(self.concertmasterlist_key)


    def setStop(self):
        self._stop = True

