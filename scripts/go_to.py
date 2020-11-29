#!/usr/bin/env python
import rospy
import time
from mrs_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

class posicao_atual:
    def __init__(self):
        self.pos_subscriber = rospy.Subscriber(
            "/uav1/control_manager/position_cmd", PositionCommand, self.callback_pos
        )
        self.dado_posicao = PositionCommand()

    def callback_pos(self, data):
        self.dado_posicao = data

def main():
	rospy.init_node("goto", anonymous = False)
	publisher = rospy.Publisher("/uav1/control_manager/reference", ReferenceStamped, queue_size = 3)
	#rospy.Subscriber("/uav1/control_manager/position_cmd", PositionCommand, funcao)
	mensagem = ReferenceStamped()
	pos_atual = posicao_atual()

	header_msg = Header()
	reference_msg = Reference()

	header_msg.frame_id = "uav1/hector_origin"
	reference_msg.position.x = -2.0
	reference_msg.position.y = 2.0
	reference_msg.position.z = 2.0

	mensagem.header = header_msg
	mensagem.reference = reference_msg
	delta = 0.01

	while abs(pos_atual.dado_posicao.position.x - reference_msg.position.x) >= delta:
		publisher.publish(mensagem)

	print("saiu da condicao")
	rospy.spin()


main()
