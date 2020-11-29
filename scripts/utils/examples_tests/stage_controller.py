#! /usr/bin/env python
# rosrun stage_ros stageros $(rospack find stage_ros)/world/willow-erratic.world
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import time


laser = LaserScan()
odometry = Odometry()


def odometry_callback(data):
    global odometry
    odometry = data


def laser_callback(data):
    global laser
    laser = data


if __name__ == "__main__":
    global odometry
    global laser

    rospy.init_node("stage_controller_node", anonymous=False)

    rospy.Subscriber("/odom", Odometry, odometry_callback)
    rospy.Subscriber("/base_scan", LaserScan, laser_callback)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    vel = Twist()
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        rospy.loginfo("Posicao X: %s; Posicao Y: %s", x, y)

        if len(laser.ranges[45 * 4 : 225 * 4]) > 0:
            if min(laser.ranges) > 0.5:
                vel.linear.x = 0.5
            else:
                vel.linear.x = 0.0
                vel.angular.z = 0.5
            pub.publish(vel)

        r.sleep()
        print(odometry.pose.pose.position.x)
