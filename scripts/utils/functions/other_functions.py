import rospy
from mrs_msgs.srv import *
from mrs_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *

from utils.functions.basic_movements import go_to
from utils.classes.ReferencePublisher import ReferencePublisher
from utils.classes.Position_Command import Position_Info
from utils.classes.BlueFoxImage import *

import time
import numpy as np

#utilizado na fase 1 --------------------------------------

def compare_bases(all_bases, base):
    existe = 0
    for known_base in all_bases:
        if (
            (abs(base[1] - known_base[1]) < 0.5)
            and (abs(base[0] - known_base[0]) < 0.5)
            or (base[0] == 0.0 and base[1] == 0.0)
        ):
            existe = 1
            print("existe ja a base")
            break
    return existe


#utilizado na fase 2 ----------------------------------------

#comparacao utilizando delta x e y, nao utilizamos mas resultados podem ser melhores
def compara_locs(distances):
    unique_sens = list()
    idx_visto = list()
    num_sens = len(distances)
    temp = False
    for i in range(num_sens):
        for j in range(i, num_sens):
            dx = distances[i][0] - distances[j][0]
            dy = distances[i][1] - distances[j][1]
            dist = np.sqrt(dx * dx + dy * dy)
            if dist < 0.3:
                x = int((distances[i][0] + distances[j][0]) / 2)
                y = int((distances[i][1] + distances[j][1]) / 2)
                unique_sens.append((x, y))
                idx_visto.append(i)
                idx_visto.append(j)
                break
    for i in range(distances):
        temp = False
        for j in idx_visto:
            if i == j:
                temp = True
                break
        if temp == False:
            unique_sens.append(distances[i])
    return unique_sens

#cria varios pontos para percorrer o tubo devagar
def divide_line(tube_start_end, k, z):
    # definindo cada um apenas para ficar mais entendivel
    # divide a reta em k vezes e retorna os pontos
    x_s = tube_start_end[0][0]
    y_s = tube_start_end[0][1]
    x_e = tube_start_end[1][0]
    y_e = tube_start_end[1][1]
    positions_list = []
    i = 1.0
    while i <= k:
        t = float(i) / float(k)
        x_destin = x_s * (1 - t) + x_e * t
        y_destin = y_s * (1 - t) + y_e * t
        positions_list.append((x_destin, y_destin, z))
        i += 1

    return positions_list

#retorna dois pontos existentes no tubo
def get_tube_points():
    return [[-3.55, 4.599], [-2.898, 1.625]]

#utilizando os dois pontos, calcula um ponto final e inicial
#tomando em consideracao o tamanho da arena
def get_tube_start_end(points):
    is_x = True
    x_1 = points[0][0]
    y_1 = points[0][1]
    x_2 = points[1][0]
    y_2 = points[1][1]
    x_test = -6
    delta_x_test = x_1 - x_2
    delta_y_test = y_1 - y_2
    inc = float(delta_y_test) / float(delta_x_test)
    my_delta_x = x_test - x_1
    y_test = y_1 + my_delta_x * inc
    if y_test > 6.3:
        is_x = False
        y_test = 6
        inc = float(1) / float(inc)
        my_delta_y = y_test - y_1
        x_test = x_1 + my_delta_y * inc

    if is_x:
        x_end = 0
        y_end = y_test + ic * 5
    else:
        y_end = 0
        x_end = x_test + inc * (-5)

    return [[x_test, y_test], [x_end, y_end]]
