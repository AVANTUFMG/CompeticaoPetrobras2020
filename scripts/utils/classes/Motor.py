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


class Motor:
    def __init__(self):
        self.sub = rospy.Subscriber(
            "/uav1/control_manager/diagnostics",
            ControlManagerDiagnostics,
            self.callback_motores,
        )
        self.motors = bool()

    def callback_motores(self, msg):
        self.motors = msg.motors
