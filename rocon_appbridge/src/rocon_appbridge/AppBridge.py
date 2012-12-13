import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import *
from demo_msgs.msg import *


class AppBridge(object):
    def __init__(self):
        self.sub ={}
        self.sub['command'] = rospy.Subscriber('command',Command,self.processCommand)

        self.pub = {}
        self.pub['status'] = rospy.Publisher('status',String)

        self.actionclient = {}
        self.actionclient['move_base'] = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        self.log("Wait for move base server")
        self.actionclient['move_base'].wait_for_server()

        self.location = {}
        self.location['kitchen'] = rospy.get_param('~kitchen',[0, 0])
        self.location['loc1'] = rospy.get_param('~loc1',[0,0])

    def spin(self):
        self.log("Ready")
        rospy.spin()

    def processCommand(self,msg):
        self.log("Received : " + str(msg))
        command = msg.command
        param = msg.param

        if command == "Goto":
            location = self.location[msg.param]
            self.send_goal(location)
        else:
            m = "Command Error : " + msg.command
            self.log(m)

    def send_goal(self,loc):
        self.log("Sending robot to : " + str(loc))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = loc[0]
        goal.target_pose.pose.position.y = loc[1]

        self.log("OK")
        self.actionclient['move_base'].sendGoal(goal)


    def log(self,msg):
        self.pub['status'].publish(msg)
        rospy.loginfo("AppBridge : " + str(msg))
