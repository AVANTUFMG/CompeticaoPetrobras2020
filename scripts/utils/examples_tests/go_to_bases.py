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

from utils.classes.Land import *
from utils.classes.Position_Command import *

# CONST_X = 1.05
# CONST_Y = 0.838716042708


rospy.init_node("go_to_bases", anonymous=True)

temp_base_locations = [(-3.20, 0, 2.5), (-6.0, 0, 2.5), (-4.6, 2.2, 1.5), (0, 0, 2.5)]

arming = bool()
mode = String()

rate = rospy.Rate(20)

message_types = [SetBool, CommandBool, SetMode, Trigger]
arguments = [1, 1, [0, "offboard"], None]


def callback_states(msg):
    global arming
    global mode
    arming = msg.armed
    mode = msg.mode


def get_pos():
    return pos.position.x, pos.position.y, pos.position.z


def wait_till_position(goal):
    pos = Position_Info()
    delta = 0.005
    print("Waiting for: {}".format(goal))
    x_aux = pos.pos.position.x
    y_aux = pos.pos.position.y
    if abs(x_aux - goal.position.x >= delta) or abs(y_aux - goal.position.y >= delta):
        return False
    return True


def go_to(publisher, pos):
    header = Header()
    reference = Reference()

    header.frame_id = "uav1/hector_origin"

    x, y, z = pos[0], pos[1], pos[2]
    reference.position = Point(x, y, z)

    msg = ReferenceStamped()
    msg.header = header
    msg.reference = reference
    publisher.publish(msg)
    while not wait_till_position(reference):
        publisher.publish(msg)


def takeoff():
    global arming
    global mode
    global services

    state_motors = Tracker()

    services = [
        "/uav1/control_manager/motors 1",
        "/uav1/mavros/cmd/arming 1",
        "/uav1/mavros/set_mode 0 offboard",
        "/uav1/uav_manager/takeoff",
    ]

    # Motors
    while not state_motors.motor:
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
    print("Waiting for {}\n".format(serviceName))
    rospy.wait_for_service(serviceName)
    try:
        run_service = rospy.ServiceProxy(serviceName, Trigger)
        print("Running {}\n\n".format(serviceName))
        resp1 = run_service()
    except rospy.ServiceException as e:
        print("Error: %s" % e)
    time.sleep(1)


def land_takeoff_routine(publisher, pos, next_pos=None):
    go_to(publisher, pos)
    print("Ready to land")
    time.sleep(2)
    land()
    time.sleep(5)
    print("Ready to take off")
    takeoff()
    time.sleep(25)


# TODO: create separate file
class Tracker:
    def __init__(self):
        self.sub_tracker = rospy.Subscriber(
            "/uav1/control_manager/diagnostics",
            ControlManagerDiagnostics,
            self.callback_tracker_motor,
        )
        self.tracker = String()
        self.motors = bool()

    def callback_tracker_motor(self, msg):
        self.tracker = msg.active_tracker
        self.motors = msg.motors


def main():
    # TODO: create separate file for publisher
    pub = rospy.Publisher(
        "/uav1/control_manager/reference", ReferenceStamped, queue_size=1
    )
    rospy.Subscriber(
        "/uav1/mavros/state", State, callback_states
    )  # subscriber para pegar os estados do drone, armado e o modo
    # para pegar estado dos motores

    Position_Command()
    Tracker()

    # rate = rospy.Rate(20)
    for position in temp_base_locations:
        land_takeoff_routine(pub, position)
        print("\n\n\nFinish one routine\n\n\n")

    rospy.spin()


main()
