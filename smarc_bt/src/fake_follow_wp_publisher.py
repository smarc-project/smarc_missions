#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


import rospy
import time
import math

from smarc_msgs.msg import GotoWaypoint
from std_msgs.msg import Float64, Header, Bool, Empty


rospy.init_node("fake_follow_wp_publisher")
update_period = 0.1
rate = rospy.Rate(1/update_period)


enable_pub = rospy.Publisher('/sam/algae_farm/enable', Bool, queue_size=1)
wp_pub = rospy.Publisher('/sam/algae_farm/wp', GotoWaypoint, queue_size=1)


# left of the real farm
center = (643740.969, 6459248.518)
# radians per wp update
rpu = math.pi/3
radius = 50

current_radians = 0

current = (center[0] + radius, center[1])

enable = Bool()
enable.data = True

# seconds
wp_update_period = 20
last_wp_update = time.time()

while not rospy.is_shutdown():
    enable_pub.publish(enable)
    wp = GotoWaypoint()
    wp.waypoint.pose.header.frame_id = 'utm'
    wp.waypoint.pose.pose.position.x = current[0]
    wp.waypoint.pose.pose.position.y = current[1]
    wp.travel_depth = 2
    wp.goal_tolerance = 2
    wp.z_control_mode = GotoWaypoint.Z_CONTROL_DEPTH
    wp.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
    wp.travel_rpm = 1000



    current_radians += rpu
    current_radians = current_radians % math.pi
    x = radius* math.cos(current_radians)
    y = radius* math.sin(current_radians)
    current = (center[0] + x, center[1] + y)

    if time.time() - last_wp_update > wp_update_period:
        wp_pub.publish(wp)
        last_wp_update = time.time()
        print("pubd x={:.2f}, y={:.2f}, rad={:.2f}".format(x,y,current_radians))

    rate.sleep()

print("Done")




