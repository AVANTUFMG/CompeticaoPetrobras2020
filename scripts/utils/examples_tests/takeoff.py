#!/usr/bin/env python

import rospy
import time
from mrs_msgs.srv import *
from mrs_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *

# Talvez usar ros::time e uma boa....
# Por que o modo offboard?
# Para motores pode ser tbm o /uav1/control_manager/motors ?

services = [
    "/uav1/control_manager/motors 1",
    "/uav1/mavros/cmd/arming 1",
    "/uav1/mavros/set_mode 0 offboard",
    "/uav1/uav_manager/takeoff",
]

message_types = [SetBool, CommandBool, SetMode, Trigger]
arguments = [1, 1, [0, "offboard"], None]
arming = bool()
mode = String()
motors = bool()


def callback_states(msg):
    global arming
    global mode
    arming = msg.armed
    mode = msg.mode


def callback_motores(msg):
    global motors
    motors = msg.motors


if __name__ == "__main__":
    global arming
    global mode
    global services
    global motors

    rospy.init_node("nodo_takeoff", anonymous=True)
    rospy.Subscriber(
        "/uav1/mavros/state", State, callback_states
    )  # subscriber para pegar os estados do drone, armado e o modo
    rospy.Subscriber(
        "/uav1/control_manager/diagnostics", ControlManagerDiagnostics, callback_motores
    )  # para pegar estado dos motores

    # Motors
    while not motors:
        serviceName = services[0].split(" ")[0]
        print("Waiting for {}\n".format(serviceName))
        rospy.wait_for_service(serviceName)
        try:
            run_service = rospy.ServiceProxy(serviceName, SetBool)
            print("Running {}\n\n".format(serviceName))
            resp1 = run_service(1)
        except rospy.ServiceException as e:
            print("Error: %s" % e)
        time.sleep(1)

    # Arming UAV
    while not arming:
        serviceName = services[1].split(" ")[0]
        print("Waiting for {}\n".format(serviceName))
        rospy.wait_for_service(serviceName)
        try:
            run_service = rospy.ServiceProxy(serviceName, CommandBool)
            print("Running {}\n\n".format(serviceName))
            resp1 = run_service(1)
        except rospy.ServiceException as e:
            print("Error: %s" % e)
        time.sleep(1)

    # Set Mode
    while mode != "OFFBOARD":
        serviceName = services[2].split(" ")[0]
        print("Waiting for {}\n".format(serviceName))
        rospy.wait_for_service(serviceName)
        try:
            run_service = rospy.ServiceProxy(serviceName, SetMode)
            print("Running {}\n\n".format(serviceName))
            resp1 = run_service(0, "offboard")
        except rospy.ServiceException as e:
            print("Error: %s" % e)
        time.sleep(1)

    # Takeoff
    serviceName = services[3].split(" ")[0]
    # print('Waiting for {}\n'.format(serviceName))
    # rospy.wait_for_service(serviceName)
    run_service = rospy.ServiceProxy(serviceName, Trigger)
    print("Running {}\n\n".format(serviceName))
    resp1 = run_service()
