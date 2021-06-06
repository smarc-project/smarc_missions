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

- meanwhile, publish GPS and have an action server to connect to
"""

import actionlib
import rospy
import time

from imc_ros_bridge.msg import PlanDB, PlanControl, PlanManeuver
from sensor_msgs.msg import NavSatFix
from smarc_msgs.msg import GotoWaypointActionFeedback, GotoWaypointResult, GotoWaypointAction, GotoWaypointGoal
from std_msgs.msg import Header


# see smarc_bt/src/imc_enums for these
PLANDB_TYPE_SUCCESS = 1
PLANDB_OP_GET_INFO = 3
PLANDB_TYPE_REQUEST = 0
PLANDB_OP_GET_STATE = 5


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


class FakeNeptus:
    def __init__(self):
        self.plandb_pub = rospy.Publisher('/lolo/imc/plan_db', PlanDB, queue_size=1)
        self.plandb_sub = rospy.Subscriber('/lolo/imc/plan_db', PlanDB, callback = self.plandb_cb)
        self.plancontrol_pub = rospy.Publisher('/lolo/imc/plan_control', PlanControl, queue_size=1)

        self.pm = self.make_plandb_msg()
        self.pm_ask = self.make_plandb_msg(ask_ack=True)

        pc = PlanControl()
        pc.type = 0
        pc.op = 0
        pc.plan_id = 'ci_plan'
        pc.flags = 1
        self.pc = pc

        self.plan_received = False

    def plandb_cb(self, msg):
        if msg.plan_id == 'ci_plan' and \
           msg.type == PLANDB_TYPE_SUCCESS and \
           msg.op == PLANDB_OP_GET_INFO:
            rospy.loginfo("Plan ACK by BT")
            self.plan_received = True


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
        while not self.plan_received and not rospy.is_shutdown():
            rospy.loginfo("Sent plan")
            self.plandb_pub.publish(self.pm)
            time.sleep(0.5)
            rospy.loginfo("Asking for ACK")
            self.plandb_pub.publish(self.pm_ask)
            time.sleep(0.5)
            if self.plan_received:
                rospy.loginfo("Stopping sending the plan")
                return

    def start_plan(self):
        # TODO wait for 'maneuver mode' instead of just sending it 5 times
        for i in range(5):
            self.plancontrol_pub.publish(self.pc)
            rospy.loginfo("Sent start")
            time.sleep(0.2)





if __name__ == '__main__':
    rospy.init_node('fake_neptus_and_user')

    # first of all, the BT needs at least a GoToWaypoint action server to connect to
    # in the setup phase.
    # we also need this here so we can send some successes to the BT easily
    fake_goto = FakeGotoServer(name='/lolo/ctrl/goto_waypoint')
    fake_emergency = FakeGotoServer(name='/lolo/ctrl/emergency_surface_action')

    # the BT will wait for _at least one_ gps before it does ANYTHING
    # make sure there is at least that one gps fix, even if it is empty
    fake_gps = FakeGPS()
    gps_timer = rospy.Timer(rospy.Duration(1), fake_gps.publish)

    # once gps and actionsrever are there, we can start neptus
    # and send a plan
    fake_neptus = FakeNeptus()


    rospy.loginfo("Waiting 2s")
    time.sleep(2)
    fake_neptus.send_and_ask_ack()

    if fake_neptus.plan_received:
        rospy.loginfo("Got ACK from BT")
        time.sleep(0.5)
        fake_neptus.start_plan()

    rospy.loginfo("Done")

    # aaaand we are done?




