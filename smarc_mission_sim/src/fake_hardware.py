#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

import actionlib
import rospy
import time

from smarc_bt.msg import GotoWaypointActionFeedback, GotoWaypointResult, GotoWaypointAction, GotoWaypointGoal
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class FakeGotoServer:
    def __init__(self, name):
        self.name = name
        self.server = actionlib.SimpleActionServer(self.name, GotoWaypointAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

        self.result = GotoWaypointResult()
        self.feedback = GotoWaypointActionFeedback()

        self.start_time = None

    def execute_cb(self, goal):
        # we just return running for some seconds and then success
        # that is all we need to get the BT to do things
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.result.reached_waypoint = False
                self.server.set_preempted(self.result, text="Preempted")
                self.start_time = None
                return

            if self.start_time is not None and time.time() - self.start_time > 5:
                self.result.reached_waypoint = True
                self.server.set_succeeded(self.result, text="Success")
                self.start_time = None
                return

            if self.start_time is None:
                rospy.loginfo("Started")
                self.start_time = time.time()


            rospy.loginfo_throttle(1, "Running")



class FakeGPS:
    def __init__(self):
        self.gps_pub = rospy.Publisher('/lolo/core/gps', NavSatFix, queue_size=1)
        self.gps = NavSatFix()
        self.gps.status.status = 0
        self.gps.latitude = 0
        self.gps.longitude = 0
        h = Header()
        h.stamp = rospy.Time.now()
        self.gps.header = h

    def publish(self, timer):
        rospy.loginfo("Published GPS")
        h = Header()
        h.stamp = rospy.Time.now()
        self.gps.header = h
        self.gps_pub.publish(self.gps)


if __name__ == '__main__':
    rospy.init_node('fake_hardware')

    # first of all, the BT needs at least a GoToWaypoint action server to connect to
    # in the setup phase.
    # we also need this here so we can send some successes to the BT easily
    fake_goto = FakeGotoServer(name='/lolo/ctrl/goto_waypoint')
    fake_emergency = FakeGotoServer(name='/lolo/ctrl/emergency_surface_action')


    # the BT will wait for _at least one_ gps before it does ANYTHING
    # make sure there is at least that one gps fix, even if it is empty
    fake_gps = FakeGPS()
    gps_timer = rospy.Timer(rospy.Duration(2), fake_gps.publish)

    rospy.loginfo("Fake GPS and action servers running...")
    rospy.spin()
