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


class Position_Info:
    def __init__(self):
        self.pos_subscriber = rospy.Subscriber(
            "/uav1/control_manager/position_cmd", PositionCommand, self.callback_pos
        )
        self.pos = PositionCommand()

    def callback_pos(self, data):
        self.pos = data
