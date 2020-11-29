#!/usr/bin/env python
import math
import time
import rospy

from utils.classes.BlueFoxImage import *
from utils.functions.basic_movements import *
from utils.functions.complex_movements import indicacao
from utils.classes.ReferencePublisher import ReferencePublisher
from utils.classes.Position_Command import Position_Info
from utils.classes.Painel import Painel

def photo_analyse():
    imagem = BlueFoxImage()
    if imagem.exist_panel():
        print("\nPainel localizado")
        numeros_encontrados = imagem.get_numbers()
        painel = Painel(numeros_encontrados[0], numeros_encontrados[1])
        print(painel)
    else:
        print("\nNao ha painel nesta base\n")

def get_distance_with_photo():
    imagem = BlueFoxImage()
    c = imagem.get_distances_platforms()
    print(c)
    return c

#compara um sinal com uma lista de sinais (sinal eh uma cor vermelha)
def red_already_exists(signal, all_signals):
    for known_signal in all_signals:
        if (
            (abs(signal[1] - known_signal[1]) < 0.4)
            and (abs(signal[0] - known_signal[0]) < 0.5)
            or (signal[0] == 0.0 and signal[1] == 0.0)
        ):
            print("\nSensor detectado anteriormente")
            return True
    return False

#tira a foto e procura se existe sinal
def search_red_in_photo(n_signals, all_signals):
    image = BlueFoxImage()
    found_red = image.get_distances_red_signals()
    new_points = []
    for red_signal in found_red:
        if not red_already_exists(red_signal, all_signals):
            n_signals += 1
            new_points.append(red_signal)
            print(
                "\n\nSinal numero: {}\nCooordenadas: {:.2f}, {:.2f}".format(
                    n_signals, red_signal[0], red_signal[1]
                )
            )
            print("Executando indicacao para o sinal {}".format(n_signals))
            indicacao()
            time.sleep(3)

    for red_signal in new_points:
        all_signals.append(red_signal)

    return n_signals, all_signals