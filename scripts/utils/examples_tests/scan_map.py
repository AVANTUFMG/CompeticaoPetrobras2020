#!/usr/bin/env python

import rospy
import time

from mrs_msgs.srv import *
from mrs_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *

from utils.classes.Position_Command import Position_Info
from utils.classes.Laser import Laser
from utils.functions.go_to import *


def main():
    rospy.init_node("map_scan", anonymous=False)
    laserObj = Laser()
    pos = Position_Info()

    pub = rospy.Publisher(
        "/uav1/control_manager/reference", ReferenceStamped, queue_size=10
    )

    while laserObj.laser.ranges == []:
        print("Waiting for laser.ranges")
        pass

    if laserObj.right_is_greater:
        #  tem que ir pra o relativo (4-distancia_da_frente, 2-distancia_da_esquerda)
        print("Right distance is greater")
        first_left_dist = laserObj.left
        back_dist = laserObj.back

        goal_x = -1 * (4 - back_dist)
        goals_y = [2 - first_left_dist, 4 - first_left_dist, 6 - first_left_dist]
        z = 3
        positions = [
            (goal_x, goals_y[0], z),
            (goal_x, goals_y[1], z),
            (goal_x, goals_y[2], z),
        ]

        for position in positions:
            go_to(pub, position)
            # tirar fotos
            # go home

    else:
        #  tem que ir pra o relativo (4-distancia_da_frente, -2+distancia_da_esquerda)
        print("wrong")
    rospy.spin()


main()
