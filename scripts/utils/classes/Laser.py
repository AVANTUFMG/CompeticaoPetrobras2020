import rospy
from sensor_msgs.msg import *


class Laser:
    def __init__(self):
        self._laser = LaserScan()
        self.sub = rospy.Subscriber(
            "/uav1/rplidar/scan", LaserScan, self.callback_laser
        )

    def callback_laser(self, data):
        self._laser = data

    # Getting distances within a 15 degrees wide field of view
    @property
    def left(self):
        dist_left = min(self.laser.ranges[165:195])
        return dist_left

    @property
    def right(self):
        dist_right = min(self.laser.ranges[525:555])
        return dist_right

    @property
    def front(self):
        current_front = self.laser.ranges[0:15] + self.laser[-15:]
        return current_front

    @property
    def back(self):
        dist_right = min(self.laser.ranges[345:375])
        return dist_right

    @property
    def laser(self):
        return self._laser

    @property
    def right_is_greater(self):
        right_is_greater = self.right > self.left
        return right_is_greater
