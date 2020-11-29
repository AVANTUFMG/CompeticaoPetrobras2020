#!/usr/bin/env python
import rospy
from mrs_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
from mavros_msgs.srv import *
from mrs_msgs.srv import *
from mrs_msgs.msg import *
from mavros_msgs.msg import *

import time

class Position_Info:
    def __init__(self):
        self.pos_subscriber = rospy.Subscriber(
            "/uav1/control_manager/position_cmd", PositionCommand, self.callback_pos
        )
        self.pos = PositionCommand()

    def callback_pos(self, data):
        self.pos = data

def in_position(goal, p_atual):
    #pos = Position_Info()
    delta = 0.001
    x_aux = p_atual.pos.position.x
    y_aux = p_atual.pos.position.y
    z_aux = p_atual.pos.position.z
    if (
        (abs(x_aux - goal.position.x) >= delta)
        or (abs(y_aux - goal.position.y) >= delta)
        or (abs(z_aux - goal.position.z) >= delta)
    ):
        time.sleep(2)
        return False
    return True
	

def main():
	rospy.init_node("go_to", anonymous = False)
	publisher = rospy.Publisher("/uav1/control_manager/reference", ReferenceStamped, queue_size = 1)
	mensagem = ReferenceStamped()
	header = Header()
	reference = Reference()
	atual = Position_Info()
	header.frame_id = "uav1/hector_origin"
	x, y, z = -3, 2, 2
	reference.position = Point(x,y,z)
	mensagem.header = header
	mensagem.reference = reference
	while not in_position(reference, atual):
		publisher.publish(mensagem)

main()