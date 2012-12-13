
import rospy
from demo_msgs.msg import *
from std_msgs.msg import String


class Kitchen(object):
    
    def __init__(self):
        self.pub = {}
        self.pub['status'] = rospy.Publisher('~status',String)

        self.sub = {}
        self.sub['order'] = rospy.Subscriber('~order',Order,self.processOrder)
        self.sub['command'] = rospy.Subscriber('~command',Command,self.processCommand)

    def spin(self):

        pub = rospy.Publisher('/command',Command)
        while not rospy.is_shutdown():
            c = Command()
            c.command ="Goto"
            c.param = "kitchen"
            rospy.sleep(4)




    def processCommand(self,msg):
        self.log('Got Command : ' + str(msg))

    def processOrder(self,msg):
        self.log('Got order : ' + str(msg))


    def log(self,msg):
        self.pub['status'].publish(msg)
        rospy.loginfo("Kitchen : " + str(msg))
