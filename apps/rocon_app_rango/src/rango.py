#!/usr/bin/env python

##############################################################################
#   
##############################################################################

import os
import threading
import roslib; roslib.load_manifest('rocon_app_rango')
import rospy
import actionlib
from std_msgs.msg import Empty
from std_msgs.msg import String
from sensor_integration_board_comms.msg import OnOffDevice
from robosem_comms.srv import AceTask
from robosem_comms.srv import AceTaskRequest
from manipulation_comms.msg import JointTrajectoryAction
from manipulation_comms.msg import JointTrajectoryGoal

##############################################################################
# Classes
##############################################################################

class MoviePlayer(threading.Thread):
    def __init__(self, movie_finished_cb):
        threading.Thread.__init__(self)
        self.movie_file = os.path.join(roslib.packages.get_pkg_dir('rocon_app_rango'), "resources", "movies", "rango.avi")
        self.movie_finished_cb = movie_finished_cb
    
    def run(self):
        '''
          Plays the movie in mplayer in a background thread."
          
          Could do better thread management of this with pid's and 
          stuff (cleaner exit if there's a problem).
        '''
        rospy.loginfo("Rango Player: starting the movie [%s]"%self.movie_file)
        #cmd = "mplayer " + self.movie_file# + " > /dev/null 2>&1"
        #cmd = "totem --fullscreen " + self.movie_file# + " > /dev/null 2>&1"
        cmd = "smplayer -fullscreen -close-at-end " + self.movie_file# + " > /dev/null 2>&1"
        os.system(cmd)
        rospy.sleep(1)
        self.movie_finished_cb()

class Rango():
    Idle = 0
    WaitingForTouch = 1
    Playing = 2
    
    def __init__(self):
        self.state = Rango.Idle
        # Global Interface : this would be better as part of an actionlib interface
        self.finished_publisher = rospy.Publisher('finished_lesson', Empty)
        self.start_lesson_subscriber = rospy.Subscriber("start_lesson", Empty, self.start_lesson_gcb)
        # Local Interface
        self.touch_sensor_subscriber = rospy.Subscriber("~touch_sensors", OnOffDevice, self.touch_sensor_lcb)
        self.disable_publisher = rospy.Publisher('~disable',String)
        self.enable_publisher = rospy.Publisher('~enable',String)
        self.arm_disabled = True
        self.arm_is_busy = False
        rospy.sleep(0.5) # pubs and subs need some time to establish connections
        self.start_time = rospy.Time.now()
        rospy.loginfo("Rango Player: initialised.")
        
    def greet(self):
        rospy.loginfo("Rango Player: greeting the muppets.")
        cmd = "espeak -v en-scottish 'Hallo muppets,'"
        os.system(cmd)
        rospy.sleep(0.3)
        cmd = "espeak -v en-scottish ' Please touch my hand if you want to watch a movie.'"
        os.system(cmd)

    def arm_action(self,task_name):
        '''
          Contacts the arm to do a full motion. Note that this is a simple action client only -
          take care not to have multiple action clients accessing the server at one time.
        '''
        self.arm_disabled = False
        self.arm_is_busy = True
        rospy.loginfo("Rango Player: requesting joint trajectory for task '%s'."%task_name)
        self.last_task_name = task_name
        try:
             rospy.wait_for_service('~ace_task',5.0)
        except rospy.ROSException:
            rospy.logerr("Rango Player: could not contact the ace task service.")
            return
        get_ace_trajectory = rospy.ServiceProxy('~ace_task', AceTask)
        request = AceTaskRequest()
        request.name = task_name
        request.repeat = 1
        response = get_ace_trajectory(request)
        if not response.exists:
            rospy.logerr("Rango Player: ace task does not exist in the trajectory database.")
            return
        rospy.loginfo("Rango Player: enabling motors.")
        self.enable_publisher.publish(String("all"))
        rospy.loginfo("Rango Player: sending joint trajectory goal for '%s'."%task_name)
        client = actionlib.SimpleActionClient('~joint_trajectory',JointTrajectoryAction)
        client.wait_for_server()
        goal = JointTrajectoryGoal()
        goal.trajectory = response.joint_trajectory
        client.send_goal(goal, self.arm_action_done_cb)
        
    def arm_action_done_cb(self, goal_status, goal_result):
        '''
          We don't want the arm action to block, so we use a callback to process
          the result instead.
        '''
        self.arm_is_busy = False
        rospy.loginfo("Rango Player: arm action completed (need better debugging here).")
        if self.last_task_name == "zero":
            self.disable_publisher.publish(String("all"))
            self.arm_disabled = True
        
    def start_lesson_gcb(self, data):
        '''
          This is a global ros callback to start the lesson 
          after the students arrive.
          
          This generations transitions from Idle->Waiting.
        '''
        rospy.loginfo("Rango Player: starting the lesson.")
        if self.state != Rango.Idle:
            # Really should have some logic going back to the concert 
            # if we're already running
            rospy.logwarn("RangoPlayer: already playing, cannot start the lesson.")
            return

        self.start_time = rospy.Time.now()
        self.state = Rango.WaitingForTouch
        self.arm_action("HOLDHANDOUT")

    def touch_sensor_lcb(self,data):
        '''
          This is a local ros callback that fires a touch sensor is touched.
          
          This generations transitions from Waiting->Playing
        '''
        if self.state != Rango.WaitingForTouch:
            return
        touched = False
        for touch in data.touch:
            if touch:
                touched = True
                break
        if not touched:
            return
        rospy.loginfo("Rango Player: received touch sensor event.")
        self.arm_action("zero")
        self.state = Rango.Playing
    
    def movie_finished_cb(self):
        '''
          Called when the movie finishes playing. It's a thread callback, not a ros callback!
          
          This generations transitions from Playing->Idle
        '''
        rospy.loginfo("Rango Player: movie finished, sending status to the concert.")
        self.arm_action("POINTSIDE")
        cmd = "espeak -v en-scottish 'Thank you for watching. The class is over. You may leave by the door.'"
        os.system(cmd)
        self.state = Rango.Idle
        self.finished_publisher.publish(Empty())

    def spin(self):
        '''
          The state control loop. This loop doesn't change the state, just checks for transitions.
          Changes in state occur in the callbacks.
        '''
        last_state = self.state
        idle_timeout = rospy.Duration(30)
        waiting_timeout = rospy.Duration(180)
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            #rospy.loginfo("Rango Player: spinning [%d][%d]"%(last_state, self.state))
            # act on state transitions
            if self.state == Rango.Idle:
                if last_state != Rango.Idle:
                    self.start_time = rospy.Time.now()
                    rospy.loginfo("Rango Player: transitioned to idle state")
                # Timeout
                elapsed = rospy.Time.now() - self.start_time
                if self.arm_disabled:
                    rospy.logdebug("Rango Player: idling [disabled][%d]"%elapsed.to_sec())
                else:
                    rospy.logdebug("Rango Player: idling [enabled][%d]"%elapsed.to_sec())
                if elapsed > idle_timeout:
                    if self.arm_disabled != True:
                        if self.arm_is_busy == False:
                            self.arm_action("zero")
            elif self.state == Rango.WaitingForTouch:
                if last_state != Rango.WaitingForTouch:
                    rospy.loginfo("Rango Player: transitioned to waiting from idle")
                    self.start_time = rospy.Time.now()
                    self.greet()
                # Timeout
                elapsed = rospy.Time.now() - self.start_time
                rospy.logdebug("Rango Player: waiting [%d]"%elapsed.to_sec())
                if elapsed > waiting_timeout:
                    rospy.loginfo("Rango Player: timed out waiting for touch to start movie playing (300s)") 
                    self.arm_action("zero")
                    self.state = Rango.Idle
                    self.finished_publisher.publish(Empty())
            elif self.state == Rango.Playing:
                if last_state != Rango.Playing:
                    rospy.loginfo("Rango Player: transitioned to playing from waiting")
                    self.start_time = rospy.Time.now()
                    MoviePlayer(self.movie_finished_cb).start()
                elapsed = rospy.Time.now() - self.start_time
                rospy.logdebug("Rango Player: playing [%d]"%elapsed.to_sec())
            last_state = self.state
        self.shutdown()
        
    def shutdown(self):
        self.disable_publisher.publish("all")
        if self.state == Rango.Playing:
            rospy.logwarn("Rango Player: shutting down dangerously without worrying about the player thread.");

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('rango')
    rango = Rango()
    #rango.arm_action("LOOKINGAROUND1")
    rango.spin()
    
