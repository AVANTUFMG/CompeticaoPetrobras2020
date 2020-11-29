#!/usr/bin/env python

import sys
import rospy
from mrs_msgs.srv import *
from mrs_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *


def callback_laser(data):
    global laser
    laser = data


def callback_pos(data):
    global pos
    pos = data


def wait_till_position(goal):
    global pos
    delta = 0.15
    x_aux = pos.position.x
    y_aux = pos.position.y
    while abs(x_aux - goal.position.x >= delta) or abs(
        y_aux - goal.position.y >= delta
    ):
        x_aux = pos.position.x
        y_aux = pos.position.y


def move_to_pos(h, ref):
    serviceName = "/uav1/control_manager/reference"
    rospy.wait_for_service(serviceName)
    try:
        goto = rospy.ServiceProxy(serviceName, ReferenceStampedSrv)
        resp1 = goto(h, ref)
        wait_till_position(ref)
    except rospy.ServiceException as e:
        print("Error: %s" % e)


def move_x(direction):
    global laser
    header = Header()
    reference = Reference()
    if direction == "forward":
        while min(laser.ranges[:30] + laser.ranges[-30:]) > 1.2:
            xi, yi, zi = get_pos()
            zi = 2.5
            xi -= 0.5
            reference.position = Point(xi, yi, zi)
            move_to_pos(header, reference)
    elif direction == "backward":
        while min(laser.ranges[330:390]) > 1.2:
            xi, yi, zi = get_pos()
            zi = 2.5
            xi += 0.5
            reference.position = Point(xi, yi, zi)
            move_to_pos(header, reference)
    else:
        print("Invalid Option")


def move_y(direction):
    header = Header()
    reference = Reference()
    dist_left = min(laser.ranges[150:210])
    dist_right = min(laser.ranges[525:555])
    if direction == "left":
        while dist_left > dist_right:
            xi, yi, zi = get_pos()
            zi = 2.5
            yi -= 1
            reference.position = Point(xi, yi, zi)
            move_to_pos(header, reference)
            dist_left = min(laser.ranges[150:210])
            dist_right = min(laser.ranges[525:555])
    elif direction == "right":
        while dist_left < dist_right:
            xi, yi, zi = get_pos()
            zi = 2.5
            yi += 0.2
            reference.position = Point(xi, yi, zi)
            move_to_pos(header, reference)
            dist_left = min(laser.ranges[150:210])
            dist_right = min(laser.ranges[525:555])
    else:
        print("Invalid Option")


def get_pos():
    return pos.position.x, pos.position.y, pos.position.z


def go_home(home):
    serviceName = "/uav1/control_manager/reference"
    rospy.wait_for_service(serviceName)
    h = Header()
    try:
        goto = rospy.ServiceProxy(serviceName, ReferenceStampedSrv)
        resp1 = goto(h, home)
        wait_till_position(home)
    except rospy.ServiceException as e:
        print("Error: %s" % e)

    serviceName = "/uav1/control_manager/eland"
    rospy.wait_for_service(serviceName)
    eland = rospy.ServiceProxy(serviceName, ReferenceStampedSrv)
    resp1 = eland()


if __name__ == "__main__":
    laser = LaserScan()
    pos = Position_Info()
    global laser
    global pos

    rospy.init_node("position_controller_node", anonymous=False)
    # rospy.Subscriber("/uav1/odometry/local_position/pose", PoseStamped, callback_pos)
    rospy.Subscriber(
        "/uav1/control_manager/position_cmd", PositionCommand, callback_pos
    )
    rospy.Subscriber("/uav1/rplidar/scan", LaserScan, callback_laser)

    header = Header()
    xi = 0
    yi = 0
    zi = 0
    while xi == 0:
        xi, yi, zi = get_pos()
    xhome, yhome, zhome = get_pos()
    while laser.ranges == []:
        pass
    while not rospy.is_shutdown():
        reference = Reference()
        if min(laser.ranges[0:30] + laser.ranges[-30:]) > 0.8:
            move_x("forward")
        else:
            dist_left = min(laser.ranges[150:210])
            dist_right = min(laser.ranges[525:555])
            if dist_left > dist_right:
                move_y("left")
            else:
                move_y("right")
            move_x("backward")
        # TODO: se a dist. na direita e atras do drone forem menor que
        # 2m entao o espaco ja foi escaneado || Ainda nao funciona
        if min(laser.ranges[525:555]) < 2 and min(laser.ranges[330:390]) < 2:
            ref_home = Refe / uav1 / odometry / local_position / poserence()
            ref_home.position = Point(xhome, yhome, zhome)
            go_home(ref_home)
