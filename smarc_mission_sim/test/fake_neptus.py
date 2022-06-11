#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


"""
1) Create a planDB message
2) Send it
3) Create a plan control state message asking for the status of the plan
4) Re-send plan until response is "got it"
5) Send start command
"""

from __future__ import print_function

import rospy
import time
import rostest
import unittest
import sys

from imc_ros_bridge.msg import PlanDB, PlanControl, PlanManeuver, PlanControlState
from std_msgs.msg import Empty


# see smarc_bt/src/imc_enums for these
PLANDB_TYPE_SUCCESS = 1
PLANDB_OP_GET_INFO = 3
PLANDB_OP_SET = 0
PLANDB_TYPE_REQUEST = 0
PLANDB_OP_GET_STATE = 5
STATE_BLOCKED = 0
STATE_READY = 1
STATE_INITIALIZING = 2
STATE_EXECUTING = 3



class FakeNeptus:
    def __init__(self):
        self.plandb_pub = rospy.Publisher('/lolo/imc/plan_db', PlanDB, queue_size=1)
        self.plandb_sub = rospy.Subscriber('/lolo/imc/plan_db', PlanDB, callback = self.plandb_cb)
        self.plancontrol_pub = rospy.Publisher('/lolo/imc/plan_control', PlanControl, queue_size=1)
        self.plancontrol_state_sub = rospy.Subscriber('lolo/imc/plan_control_state', PlanControlState, callback = self.plan_control_state_cb)

        self.pm = self.make_plandb_msg()
        self.pm_ask = self.make_plandb_msg(ask_ack=True)

        pc = PlanControl()
        pc.type = 0
        pc.op = 0
        pc.plan_id = 'ci_plan'
        pc.flags = 1
        self.pc = pc

        self.plan_received = False
        self.vehicle_state = STATE_BLOCKED
        self.plan_complete = False
        self.current_plan = None

    def plandb_cb(self, msg):
        if msg.plan_id == 'ci_plan' and \
           msg.type == PLANDB_TYPE_SUCCESS and \
           msg.op == PLANDB_OP_GET_STATE:
            rospy.loginfo("Plan ACK by BT")
            self.plan_received = True

    def plan_control_state_cb(self, msg):
        self.vehicle_state = msg.state

        if msg.plan_id == 'Mission complete' and msg.plan_progress == 100.0:
            self.plan_complete = True

    def make_plandb_msg(self, ask_ack=False):
        if ask_ack:
            pm_ask = PlanDB()
            pm_ask.type = PLANDB_TYPE_REQUEST
            pm_ask.op = PLANDB_OP_GET_STATE
            return pm_ask

        # this is a copy of the bagy file
        pm = PlanDB()
        pm.type = 0
        pm.op = 0
        pm.request_id = 7221
        pm.plan_id = 'ci_plan'
        pm.plan_spec.plan_id = 'ci_plan'
        pm.plan_spec.start_man_id = 'Goto1'

        m1 = PlanManeuver()
        m1.maneuver_id = 'Goto1'
        m1.maneuver.maneuver_name = 'goto'
        m1.maneuver.maneuver_imc_id = 450
        m1.maneuver.timeout = 10000
        m1.maneuver.lat = 1.03448527225
        m1.maneuver.lon = 0.319722020249
        m1.maneuver.z = 2.0
        m1.maneuver.z_units = 1
        m1.maneuver.speed = 1.0
        m1.maneuver.speed_units = 0
        m1.maneuver.roll = 0.0
        m1.maneuver.pitch = 0.0
        m1.maneuver.yaw = 0.0
        m1.maneuver.custom_string = ''

        m2 = PlanManeuver()
        m2.maneuver_id = 'Goto2'
        m2.maneuver.maneuver_name = 'goto'
        m2.maneuver.maneuver_imc_id = 450
        m2.maneuver.timeout = 10000
        m2.maneuver.lat = 1.03448106229
        m2.maneuver.lon = 0.319730259404
        m2.maneuver.z = 2.0
        m2.maneuver.z_units = 1
        m2.maneuver.speed = 1.0
        m2.maneuver.speed_units = 0
        m2.maneuver.roll = 0.0
        m2.maneuver.pitch = 0.0
        m2.maneuver.yaw = 0.0
        m2.maneuver.custom_string = ''

        pm.plan_spec.maneuvers.append(m1)
        pm.plan_spec.maneuvers.append(m2)

        pm.plan_spec_md5 =  [25, 130, 99, 115, 118, 138, 1, 50, 143, 238, 35, 61, 154, 97, 217, 92]
        pm.plandb_information.plan_id = ''
        pm.plandb_information.plan_size = 0
        pm.plandb_information.change_time = 0.0
        pm.plandb_information.change_sid = 0
        pm.plandb_information.change_sname = ''
        pm.plandb_information.md5 = []
        pm.plandb_state.plan_count = 0
        pm.plandb_state.plan_size = 0
        pm.plandb_state.change_time = 0.0
        pm.plandb_state.change_sid = 0
        pm.plandb_state.change_sname = ''
        pm.plandb_state.md5 = []
        pm.plandb_state.plans_info = []
        return pm

    def send_and_ask_ack(self):
        i = 0
        while not self.plan_received and not rospy.is_shutdown():
            rospy.loginfo("Sent plan")
            self.plandb_pub.publish(self.pm)
            time.sleep(2)
            rospy.loginfo("Asking for ACK")
            # try a bunch of times to send the plan. 
            # the CI online is really fiddly for some reason, this runs smoothly local...
            for j in range(10):
                if self.plan_received:
                    rospy.loginfo("Stopping sending the plan")
                    return
                rospy.loginfo("Sending pm")
                self.plandb_pub.publish(self.pm_ask)
                time.sleep(0.5)

    def start_plan(self):
        while not self.vehicle_state == STATE_EXECUTING and not rospy.is_shutdown():
            self.plancontrol_pub.publish(self.pc)
            rospy.loginfo("Sent start")
            time.sleep(0.2)
            if self.vehicle_state == STATE_EXECUTING:
                rospy.loginfo("Mission started")
                return


class TestMonitorPlan(unittest.TestCase):
    def __init__(self, *args):
        super(TestMonitorPlan, self).__init__(*args)

        self.complete_sub = rospy.Subscriber('lolo/core/mission_complete', Empty, callback=self.mission_complete_cb)
        self.mission_complete_msg_received = False

        self.heartbeat_sub = rospy.Subscriber('lolo/core/heartbeat', Empty, callback=self.heartbeat_cb)
        self.heartbeats_received = 0

    def mission_complete_cb(self, msg):
        self.mission_complete_msg_received = True

    def heartbeat_cb(self, msg):
        self.heartbeats_received += 1

    def test_monitor_plan(self):
        init_time = time.time()

        # wait for the latlon_to_utm service to exist before we send a mission
        # to the BT. Normally this waiting is done by launching the BT last
        # in the GUI.
        service_exists = False
        rospy.loginfo("Waiting for lat_lon_to_utm services")
        while not service_exists:
            time.sleep(1)
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
                    continue

        rospy.loginfo("Checking if the services exist")
        self.assert_(service_exists)

        # wait for the BT to come alive too
        # we can listen to its heartbeat
        rospy.loginfo("Waiting for BT heartbeat")
        while not rospy.is_shutdown() and self.heartbeats_received < 3:
            time.sleep(1)
        rospy.loginfo("BT is living!")



        # once gps and actionsrever are there, we can start neptus
        fake_neptus = FakeNeptus()
        # and send a plan
        last_hb_count = self.heartbeats_received
        t0 = time.time()
        fake_neptus.send_and_ask_ack()

        diff = self.heartbeats_received - last_hb_count
        time_diff = time.time() - t0
        self.assert_(diff > 0, "The BT stopped hearbeats after send_and_ask_ack, hb_diff:{}, time_diff:{}\n Likely a crash of the BT...run local CI!".format(diff, time_diff))


        self.assert_(fake_neptus.plan_received, "Plan not received by the BT!")

        if fake_neptus.plan_received:
            rospy.loginfo("Got ACK from BT")
            time.sleep(0.5)
            fake_neptus.start_plan()

        # after some 5x#waypoints seconds, the mission should be complete
        # hardcoded 2 waypoints right now
        for i in range(10):
            rospy.loginfo("Waiting for the mission...")
            time.sleep(1)
            if rospy.is_shutdown():
                break


        # XXX this part can be changed to test for different things depending on the mission
        # sent and the action servers being used.
        # for the fake actions, just a simple check of iterating over points is good enough
        timeout = time.time() + 100.0
        completion_time = None
        while not rospy.is_shutdown() and time.time() < timeout:
            if fake_neptus.plan_complete:
                rospy.loginfo("Mission complete!")
                completion_time = time.time()
                break
            else:
                rospy.loginfo("Mission running, status:{}".format(fake_neptus.vehicle_state))
            time.sleep(0.5)

        rospy.loginfo("Checking if the mission was completed in time")
        # triple check the same thing
        self.assert_(fake_neptus.plan_complete, "Mission complete not received!")
        self.assert_(completion_time is not None, "Completion time is unknown!")
        self.assert_(completion_time <= timeout+1, "Completion time is after the timeout!")

        time.sleep(1)
        self.assert_(self.mission_complete_msg_received, "core/mission_complete not received after mission!")



        done_time = time.time()
        rospy.loginfo("Done in {:.2f} seconds".format(done_time-init_time))




if __name__ == '__main__':
    rospy.init_node('fake_neptus_and_user')
    rostest.rosrun('smarc_mission_sim',
                   'fake_neptus',
                   TestMonitorPlan,
                   sys.argv)





