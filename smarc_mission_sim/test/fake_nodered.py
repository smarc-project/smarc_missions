#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

"""
1) Create a MissionControl message
2) Send it
3) Listen to feedback
4) Re-send until feedback = plan
5) Send start, pause, stop
6) Check feedback
"""


from __future__ import print_function

import rospy
import time
import rostest
import unittest
import sys
import random

from smarc_msgs.msg import MissionControl, GotoWaypoint
from std_msgs.msg import Empty

def make_random_plan(mission_name="test mission", timeout=600):
    mc = MissionControl()
    rnd = "".join([random.choice("testmission") for i in range(10)])
    mc.name = "TEST-- {} -{}".format(mission_name, rnd)
    mc.hash = "nohash"
    mc.timeout = timeout
    mc.command = MissionControl.CMD_SET_PLAN

    for i in range(2):
        wp = GotoWaypoint()
        wp.pose.header.frame_id = 'latlon'
        wp.goal_tolerance = 5
        wp.z_control_mode = GotoWaypoint.Z_CONTROL_DEPTH
        wp.travel_depth = 3
        wp.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
        wp.travel_rpm = 2000+i
        wp.lat = 43.931771
        wp.lon = 15.441733
        wp.name = "test point {}".format(i)
        mc.waypoints.append(wp)

    return mc



class TestMonitorPlan(unittest.TestCase):
    def __init__(self, *args):
        super(TestMonitorPlan, self).__init__(*args)
        # and one to store the feedback we get back
        self.mc_fb = None

        mc_topic = '/lolo/smarc_bt/mission_control'
        self.mc_pub = rospy.Publisher(mc_topic, MissionControl, queue_size=1)
        self.mc_sub = rospy.Subscriber(mc_topic, MissionControl, callback=self.mc_cb, queue_size=1)

        self.heartbeat_sub = rospy.Subscriber('lolo/core/heartbeat', Empty, callback=self.heartbeat_cb)
        self.heartbeats_received = 0

    def mc_cb(self, msg):
        if msg.command == MissionControl.CMD_IS_FEEDBACK:
            self.mc_fb = msg

    def heartbeat_cb(self, msg):
        self.heartbeats_received += 1


    def wait_for_state(self, state, mc, timeout=None):
        rate = rospy.Rate(5)
        state_str = ["RUNNING","STOPPED","PAUSED","EMERGENCY","RECEIVED","COMPLETED"]
        started = time.time()
        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for {} for the sent plan {}".format(state_str[state], mc.name))
            rate.sleep()
            if timeout is not None and time.time() - started >= timeout:
                rospy.loginfo("Timeout reached")
                return False
            if self.mc_fb is None:
                rospy.loginfo("Feedback mission control was none?")
                continue
            if self.mc_fb.name != mc.name:
                rospy.loginfo("Got feedback for a different mission?")
                continue
            if self.mc_fb.plan_state != state:
                rospy.loginfo("Got {} instead?".format(state_str[mc.plan_state]))
                continue
            return True



    def test_monitor_plan(self):
        init_time = time.time()
        def log(s):
            rospy.loginfo("Elapsed:{:.1f} -- {}".format(time.time()-init_time, s))

        rate = rospy.Rate(1)

        # wait for the latlon_to_utm service to exist before we send a mission
        # to the BT. Normally this waiting is done by launching the BT last
        # in the GUI.
        log("Checking if the services exist")
        service_exists = False
        log("Waiting for lat_lon_to_utm services")
        while not service_exists:
            try:
                rospy.wait_for_service('lolo/dr/lat_lon_to_utm', timeout=1)
                service_exists = True
                break
            except:
                try:
                    rospy.wait_for_service('lolo/lat_lon_to_utm', timeout=1)
                    service_exists = True
                    break
                except:
                    rate.sleep()
                    continue
        self.assert_(service_exists)

        # wait for the BT to come alive too
        # we can listen to its heartbeat
        log("Waiting for BT heartbeat")
        while not rospy.is_shutdown() and self.heartbeats_received < 3:
            log("Got {} heartbeats".format(self.heartbeats_received))
            rate.sleep()
        log("Got {} heartbeats".format(self.heartbeats_received))
        log("BT is living!")

        # now the BT is ready to rock, we send our mission to it
        # make a simple  mission
        mc = make_random_plan(mission_name="should pass", timeout=600)
        received = False
        while not received:
            log("Sending first plan to BT")
            self.mc_pub.publish(mc)
            # and then we start checking the feedback
            received = self.wait_for_state(MissionControl.FB_RECEIVED, mc, timeout=2)
            rate.sleep()
        log("BT Got the first plan!")


        mc.command = MissionControl.CMD_START
        started = False
        while not started:
            log("Sending start to BT")
            self.mc_pub.publish(mc)
            started = self.wait_for_state(MissionControl.FB_RUNNING, mc, timeout=2)
            rate.sleep()
        log("BT is running the plan !")

        # and then the same, but wait for complete now
        self.wait_for_state(MissionControl.FB_COMPLETED, mc)
        log("BT completed the plan !")


        # cool, now we can test for a timeout emergency
        mc = make_random_plan(mission_name="shouldfail", timeout=10)
        mc.command = MissionControl.CMD_SET_PLAN
        log("Sending a short timeout mission to BT")
        self.mc_pub.publish(mc)
        self.wait_for_state(MissionControl.FB_RECEIVED, mc)
        log("BT Got the failing plan!")

        log("Sending start to BT")
        mc.command = MissionControl.CMD_START
        self.mc_pub.publish(mc)
        self.wait_for_state(MissionControl.FB_RUNNING, mc)
        log("BT is running the plan !")

        # and then the same, but wait for emergency now
        self.wait_for_state(MissionControl.FB_EMERGENCY, mc)
        log("BT got into emergency as expected!")

        # if we reached this point without timeouts and errors, we good.
        # this assert is only needed for melodic i think...
        self.assert_(True)
        log("Done!")










if __name__ == "__main__":
    rospy.init_node('fake_nodered')

    rostest.rosrun('smarc_mission_sim',
                   'fake_nodered',
                   TestMonitorPlan,
                   sys.argv)

