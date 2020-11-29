#!/usr/bin/env python

import rospy
from mrs_msgs.srv import *
from mrs_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *

from utils.functions.basic_movements import *
from utils.classes.Laser import Laser
from utils.classes.ReferencePublisher import ReferencePublisher
from utils.classes.Position_Command import Position_Info
from utils.functions.control_with_vision import *
from utils.functions.other_functions import *

import math
import time
import numpy as np


def main():
    rospy.init_node("tube_scan", anonymous=False)

    pub = ReferencePublisher()
    laser = Laser()
    two_points = get_tube_points()
    iterations = 1.0
    divide = 10.0
    height = 0.7
    contador_sinais_vermelhos = 0
    todos_sinais = []

    start_end = get_tube_start_end(two_points)
    tube_start = (start_end[0][0], start_end[0][1], height)
    go_to(pub.pub_reference, tube_start)
    time.sleep(3)

    lista_de_posicoes = divide_line(start_end, divide, height)

    for posicao in lista_de_posicoes:
        go_to(pub.pub_reference, posicao)
        time.sleep(4)
        contador_sinais_vermelhos, todos_sinais = search_red_in_photo(contador_sinais_vermelhos, todos_sinais)
        if laser.left <= 1.80:
            break
        time.sleep(1)

    base_costeira = (0, 0, 1.75)
    centro_safe = (-3, 3, 0.65)
    go_to(pub.pub_reference, centro_safe)
    time.sleep(2)
    go_to(pub.pub_reference, base_costeira)
    time.sleep(10)
    land()


main()
