#!/usr/bin/env python

import rospy
import time

from mrs_msgs.srv import *
from mrs_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *


from utils.classes.Laser import Laser
from utils.classes.ReferencePublisher import ReferencePublisher

from utils.classes.Position_Command import Position_Info
from utils.classes.BlueFoxImage import *
from utils.functions.other_functions import compare_bases
from utils.functions.basic_movements import *
from utils.functions.complex_movements import *
from utils.functions.control_with_vision import get_distance_with_photo

import cv2
import numpy
import math


### /// "GLOBAL" /// ###

bases_total = []
goals_x = [-1, -2, -5.5, -5.5]
goals_y = [3.5, 5, 2, 5.3]
z = 1.8
positions = [
    (goals_x[0], goals_y[0], z),
    (goals_x[1], goals_y[1], z),
    (goals_x[2], goals_y[2], z),
    (goals_x[3], goals_y[3], z),
]
base_s_1 = [-0.15, 6, 2.5]
base_s_2 = [-2.85, 0, 2.5]
bases_total.append(base_s_1)
bases_total.append(base_s_2)

### /// -------- /// ###


def test():
    rospy.init_node("map_scan", anonymous=False)
    # publisher = ReferencePublisher()
    # base_photo = taking_photo()
    centroide = BlueFoxImage()
    centro = centroide.angle_scan()
    print("Centro: {}".format(centro[1][0]))
    # base_teste = []
    # posicao = Position_Info()
    # base_teste.append((-6.0, 0.0, 1.5))
    # base_aux = (base_photo[0][0],  base_photo[0][1], 1.5)
    # go_to_base_and_align(publisher, base_aux)
    # base_aux = (posicao.pos.position.x, posicao.pos.position.y, 1.5)
    # print("Base teste:{}\n Base aux:{}".format(base_teste, base_aux))
    # if not compare_bases(base_teste, base_aux):
    #    print("base difere")
    # else:
    #    print("mesma base")


def main():
    rospy.init_node("map_scan", anonymous=False)
    laserObj = Laser()
    publisher = ReferencePublisher()
    pos_atual = Position_Info()
    altura_nav = 2.2
    while laserObj.laser.ranges == []:
        print("Waiting for laser.ranges")
        pass

    for position in positions:
        go_to(publisher.pub_reference, position)
        time.sleep(1)
        bases_photo = get_distance_with_photo()
        for base_p in bases_photo:
            existencia = -1
            existencia = compare_bases(bases_total, base_p)
            if existencia == 1:
                break
            elif existencia == 0:
                base_aux = (base_p[0], base_p[1], altura_nav)
                bases_total.append(base_aux)
            else:
                print("Erro no compare_bases existe = {}".format(existencia))

    print("Todas Bases: {}".format(bases_total))

    # Indo para as bases encontradas
    bases_visitadas = []
    for base in bases_total:
        go_to_base_and_align(publisher, base)
        base_aux = (
            pos_atual.pos.position.x,
            pos_atual.pos.position.y,
            pos_atual.pos.position.z,
        )
        print(base_aux)
        if (compare_bases(bases_visitadas, base_aux) == 0) or bases_visitadas == []:
            bases_visitadas.append(base_aux)
            go_to(publisher.pub_reference, (base_aux[0], base_aux[1], 0.0))
            time.sleep(4)
            go_to(publisher.pub_reference, (base_aux[0], base_aux[1], 2.0))
            time.sleep(1)
        else:
            continue

        if len(bases_visitadas) >= 5:
            break

    # Retornando a base
    go_to(publisher.pub_reference, (-3, 3, 2.5))
    go_home(publisher)
    land()

    rospy.spin()


# test()
main()
