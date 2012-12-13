import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from move_base_msgs.msg import *
from demo_msgs.msg import *


class AppBridge(object):
    def __init__(self):
        self.sub ={}
        self.sub['command'] = rospy.Subscriber('command',Command,self.processCommand)
        self.sub['order'] = rospy.Subscriber('order',Command,self.processOrder)

        self.pub = {}
        self.pub['status'] = rospy.Publisher('status',String)
        self.pub['move_turtle'] = rospy.Publisher('/mobile_base/commands/velocity',Twist)

        self.actionclient = {}
        self.actionclient['move_base'] = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        self.log("Wait for move base server")
#        self.actionclient['move_base'].wait_for_server()

        self.location = {}
        self.location['kitchen'] = rospy.get_param('~kitchen',[0, 0])
        self.location['loc1'] = rospy.get_param('~loc1',[0,0])

    def spin(self):
        self.log("Ready")
        rospy.spin()

    def processOrder(self,msg):
        t = Twist()
        t.angular.z = 1.0

        self.log("Received : " + str(msg))
        self.pub['move_turtle'].publish(t)

    def processCommand(self,msg):
        self.log("Received : " + str(msg))
        command = msg.command
        param = msg.param

        t = Twist()
        t.angular.z = 1.0

        self.log("Received : " + str(msg))
        self.pub['move_turtle'].publish(t)
        """
        if command == "Goto":
            location = self.location[msg.param]
            self.send_goal(location)
        else:
            m = "Command Error : " + msg.command
            self.log(m)
        """

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
