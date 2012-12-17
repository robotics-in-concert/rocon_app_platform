#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_app_manager/concert_client/LICENSE
#

import rospy
import redis
import threading
# It polls redis server to discover currently connected concertmaster
# and notifies concert client


class ConcertMasterDiscovery(threading.Thread):
    params = {}

    def __init__(self, hub_client, key, processNewMaster):
        threading.Thread.__init__(self)
        self.hub_client = hub_client
        self.concertmasterlist_key = key
        self.processNewMaster = processNewMaster
        self._stop = False
        self.masterlist = []

    def run(self):
        rospy.loginfo("Concert Client : concert discovery has started")

        while not rospy.is_shutdown() and not self._stop:
            try:
                discovered_masterlist = self.getMasterList()
                self.processNewMaster(discovered_masterlist)
                rospy.sleep(3)
            except redis.ConnectionError as e:
                # This just drops out - we probably need failure recovery somewhere
                rospy.logwarn("Concert Client : lost connection to the hub [%s]" % str(e))
                break
        rospy.loginfo("Concert Client : concert discovery has stopped")

    def getMasterList(self):
        return self.hub_client.getValues(self.concertmasterlist_key)

    def set_stop(self):
        self._stop = True
