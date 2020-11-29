#!/usr/bin/env python
import math
import time
import rospy

from mrs_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
from mavros_msgs.srv import *
from mrs_msgs.srv import *
from mrs_msgs.msg import *
from mavros_msgs.msg import *

from utils.classes.Tracker import Tracker
from utils.classes.Position_Command import Position_Info
from utils.classes.Motor import Motor
from utils.classes.MavrosState import MavrosState


def go_to(publisher, pos):
    header = Header()
    reference = Reference()
    msg = ReferenceStamped()
    pos_atual = Position_Info()

    header.frame_id = "uav1/hector_origin"

    x, y, z = pos[0], pos[1], pos[2]
    reference.position = Point(x, y, z)

    msg.header = header
    msg.reference = reference
    publisher.publish(msg)
    while not in_position(reference, pos_atual):
        publisher.publish(msg)


def in_position(goal, p_atual):
    pos = Position_Info()
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

def land():
    services = [
        "/uav1/uav_manager/land",
        "/uav1/control_manager/motors 0",
        "/uav1/mavros/cmd/arming 0",
    ]

    state_tracker = Tracker()

    message_types = [SetBool, CommandBool, SetMode, Trigger]
    arguments = [1, 1, [0, "offboard"], None]

    first_call = 0

    # Land
    while (
        state_tracker.tracker != "LandoffTracker"
        and state_tracker.tracker is not "NullTracker"
    ) or first_call == 0:
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
    while state_tracker.tracker != "NullTracker":
        time.sleep(2)

    while state_tracker.motors:
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

def takeoff():

    mavros_state = MavrosState()
    motors = Motor()
    state_trackers = Tracker()

    services = [
        "/uav1/control_manager/motors 1",
        "/uav1/mavros/cmd/arming 1",
        "/uav1/mavros/set_mode 0 offboard",
        "/uav1/uav_manager/takeoff",
    ]

    # Motors
    while not motors.motors:
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
    while not mavros_state.armed:
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
    while mavros_state.mode != "OFFBOARD":
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
    print("Waiting for {}\n".format(serviceName))
    rospy.wait_for_service(serviceName)
    try:
        run_service = rospy.ServiceProxy(serviceName, Trigger)
        print("Running {}\n\n".format(serviceName))
        resp1 = run_service()
    except rospy.ServiceException as e:
        print("Error: %s" % e)

    cnt = 0
    while state_trackers.tracker != "MpcTracker":
        if cnt < 8:
            print("taking off...")
            cnt += 1
            time.sleep(2)
        else:
            land()
            takeoff()