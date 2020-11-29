import rospy
from std_msgs.msg import *
from mrs_msgs.msg import *


class Tracker:
    def __init__(self):
        self.sub_tracker = rospy.Subscriber(
            "/uav1/control_manager/diagnostics",
            ControlManagerDiagnostics,
            self.callback_tracker_motor,
        )
        self.tracker = String()
        self.motors = bool()

    def callback_tracker_motor(self, msg):
        self.tracker = msg.active_tracker
        self.motors = msg.motors
