import rospy
from mrs_msgs.msg import *


class ReferencePublisher:
    def __init__(self, topic="/uav1/control_manager/reference"):
        self.topic = topic
        self._pub = rospy.Publisher(topic, ReferenceStamped, queue_size=10)

    @property
    def pub_reference(self):
        return self._pub

    def publish_once(self, msg):
        try:
            self._pub.publish(msg)
        except:
            print("Error publishing {}".format(msg))

    def __str__(self):
        return self.topic
