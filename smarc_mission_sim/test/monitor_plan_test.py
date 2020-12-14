#!/usr/bin/env python

from __future__ import print_function

PKG = 'smarc_mission_sim'
NAME = 'monitor_plan_test'

import sys
import unittest
import time

import rospy
import rostest

from imc_ros_bridge.msg import PlanControlState

class TestMonitorPlan(unittest.TestCase):

    def __init__(self, *args):
        super(TestMonitorPlan, self).__init__(*args)
        self.success = False
        self.failure = False

    def callback(self, msg):
        print(rospy.get_caller_id(), "I heard %s"%msg.plan_id)
        self.failure = msg.plan_id == "SAFETY FALLBACK"
        self.success = msg.plan_id == "Mission complete"

    def test_monitor_plan(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.Subscriber("/lolo/imc/plan_control_state", PlanControlState, self.callback)
        timeout_t = time.time() + 100.0 #100 seconds
        while not rospy.is_shutdown() and not self.success and not self.failure and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success)
        self.assert_(not self.failure)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestMonitorPlan, sys.argv)


