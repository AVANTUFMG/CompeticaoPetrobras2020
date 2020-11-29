#!/usr/bin/env python

import rospy
from mrs_msgs.srv import *
from mrs_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *

from utils.functions.basic_movements import go_to, land
from utils.functions.complex_movements import go_home, align
from utils.classes.ReferencePublisher import ReferencePublisher
from utils.classes.BlueFoxImage import *
from utils.functions.control_with_vision import photo_analyse

import math
import time
import numpy as np

# VALORES DAS PLATAFORMAS
# 7 4 -> -1.25 2
# 6 7 -> -2.25 5
# 2 2 -> -6.25 0
# -0.15, 6, 2.5
# -3, 0, 2.5
# tirar do loop do blue fox

def main():
    rospy.init_node("paineis", anonymous=False)
    pub = ReferencePublisher()
    z = 2.2
    bases = [(-0.15, 6, z), (-3.25, 0, z), (-1.25, 2, z), (-2.25, 5, z), (-6.25, 0, z)]

    for base in bases:
        print("Objetivo: {}".format(base))
        go_to(pub.pub_reference, base)
        time.sleep(2)
        go_to(pub.pub_reference, (base[0], base[1], 1))
        time.sleep(2)
        align(pub.pub_reference)
        go_to(pub.pub_reference, (base[0], base[1], 0.3))
        time.sleep(2)

        photo_analyse()
        time.sleep(1)

        go_to(pub.pub_reference, (base[0], base[1], 2.2))
        time.sleep(1)

    centro_safe = (-3, 3, 1.2)
    go_to(pub.pub_reference, centro_safe)
    time.sleep(2)
    go_home(pub)
    time.sleep(2)
    land()


main()
