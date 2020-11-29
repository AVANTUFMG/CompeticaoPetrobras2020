#!/usr/bin/env python

import time
import rospy
from std_srvs.srv import *
from mavros_msgs.srv import *
from mrs_msgs.msg import *
from std_msgs.msg import *


# O unico jeito que achei pra ver se ta land ou o que e olhando o status dos trackers, ver a seguinte doc:
# https://github.com/ctu-mrs/mrs_uav_managers/blob/master/config/default/uav_manager.yaml

services = [
    "/uav1/uav_manager/land",
    "/uav1/control_manager/motors 0",
    "/uav1/mavros/cmd/arming 0",
]

message_types = [SetBool, CommandBool, SetMode, Trigger]
arguments = [1, 1, [0, "offboard"], None]
tracker = String()
motors = bool()


def callback_tracker(msg):
    global tracker
    global motors
    tracker = msg.active_tracker
    motors = msg.motors


rospy.init_node("nodo_land", anonymous=True)
rospy.Subscriber(
    "/uav1/control_manager/diagnostics", ControlManagerDiagnostics, callback_tracker
)
first_call = 0


# Land
while tracker != "LandoffTracker" or first_call == 0:
    first_call = 1
    serviceName = services[0].split(" ")[0]
    print("Waiting for {}\n".format(serviceName))
    rospy.wait_for_service(serviceName)
    try:
        run_service = rospy.ServiceProxy(serviceName, Trigger)
        print("Running {}\n\n".format(serviceName))
        resp1 = run_service()
    except rospy.ServiceException as e:
        print("Error: %s" % e)
    time.sleep(1)


# Desligando Motors
while tracker != "NullTracker":
    time.sleep(2)

while motors:
    serviceName = services[1].split(" ")[0]
    print("Waiting for {}\n".format(serviceName))
    rospy.wait_for_service(serviceName)
    try:
        run_service = rospy.ServiceProxy(serviceName, SetBool)
        print("Running {}\n\n".format(serviceName))
        resp1 = run_service(0)
    except rospy.ServiceException as e:
        print("Error: %s" % e)
    time.sleep(1)


# Arming
serviceName = services[2].split(" ")[0]
print("Waiting for {}\n".format(serviceName))
rospy.wait_for_service(serviceName)
try:
    run_service = rospy.ServiceProxy(serviceName, CommandBool)
    print("Running {}\n\n".format(serviceName))
    resp1 = run_service(0)
except rospy.ServiceException as e:
    print("Error: %s" % e)
