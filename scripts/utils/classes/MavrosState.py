import rospy
from mrs_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
from mavros_msgs.srv import *
from mrs_msgs.srv import *
from mrs_msgs.msg import *
from mavros_msgs.msg import *

import time


class MavrosState:
    def __init__(self):
        self.sub = rospy.Subscriber("/uav1/mavros/state", State, self.callback_states)
        self.armed = bool()
        self.mode = String()

    def callback_states(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode
