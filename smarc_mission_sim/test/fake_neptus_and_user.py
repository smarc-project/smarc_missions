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

import rospy
import time

from imc_ros_bridge.msg import PlanDB, PlanControl, PlanManeuver
from sensor_msgs.msg import NavSatFix

if __name__ == '__main__':
    rospy.init_node('fake_neptus_and_user')

    # the BT will wait for _at least one_ gps before it does ANYTHING
    # make sure there is at least that one gps fix, even if it is empty
    gps_pub = rospy.Publisher('/lolo/core/gps', NavSatFix, queue_size=1)
    gps = NavSatFix()
    gps_pub.publish(gps)

    plandb_pub = rospy.Publisher('/lolo/imc/plan_db', PlanDB, queue_size=1)

#>>>> 1) Create a planDB message
# 2) Send it
# 3) Create a plan control state message asking for the status of the plan
# 4) Re-send plan until response is "got it"
# 5) Send start command
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
    m1.maneuver.lat: 1.03448527225
    m1.maneuver.lon: 0.319722020249
    m1.maneuver.z: 2.0
    m1.maneuver.z_units: 1
    m1.maneuver.speed: 1.0
    m1.maneuver.speed_units: 0
    m1.maneuver.roll: 0.0
    m1.maneuver.pitch: 0.0
    m1.maneuver.yaw: 0.0
    m1.maneuver.custom_string: ''

    m2 = PlanManeuver()
    m2.maneuver_id = 'Goto2'
    m2.maneuver.maneuver_name = 'goto'
    m2.maneuver.maneuver_imc_id = 450
    m2.maneuver.timeout = 10000
    m2.maneuver.lat: 1.03448106229
    m2.maneuver.lon: 0.319730259404
    m2.maneuver.z: 2.0
    m2.maneuver.z_units: 1
    m2.maneuver.speed: 1.0
    m2.maneuver.speed_units: 0
    m2.maneuver.roll: 0.0
    m2.maneuver.pitch: 0.0
    m2.maneuver.yaw: 0.0
    m2.maneuver.custom_string: ''

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


    # see smarc_bt/src/imc_enums for these
    PLANDB_TYPE_SUCCESS = 1
    PLANDB_OP_GET_INFO = 3
    PLANDB_TYPE_REQUEST = 0
    PLANDB_OP_GET_STATE = 5

# 3) Create a plan control state message asking for the status of the plan
# 4) Re-send plan until response is "got it"
# 5) Send start command
    pm_ask = PlanDB()
    pm_ask.type = PLANDB_TYPE_REQUEST
    pm_ask.op = PLANDB_OP_GET_STATE


    plan_received = False
    def plandb_cb(msg):
        if msg.plan_id == 'ci_plan' and \
           msg.type == PLANDB_TYPE_SUCCESS and \
           msg.op == PLANDB_OP_GET_INFO:
            rospy.loginfo("Plan received by BT")
            global plan_received
            plan_received = True

    plandb_sub = rospy.Subscriber('/lolo/imc/plan_db', PlanDB, callback = plandb_cb)
    rospy.loginfo("Waiting 2s")
    time.sleep(2)
    while not plan_received and not rospy.is_shutdown():
        rospy.loginfo("Sent plan")
        plandb_pub.publish(pm)
        time.sleep(0.5)
        rospy.loginfo("Asking for ACK")
        plandb_pub.publish(pm_ask)
        time.sleep(0.5)


    if plan_received:
        rospy.loginfo("Got ACK from BT")
        time.sleep(0.5)
    # 5) Send start command
        # the "start button"
        pc = PlanControl()
        pc.type = 0
        pc.op = 0
        pc.plan_id = 'ci_plan'
        pc.flags = 1
        plancontrol_pub = rospy.Publisher('/lolo/imc/plan_control', PlanControl, queue_size=1)
        for i in range(5):
            plancontrol_pub.publish(pc)
            rospy.loginfo("Sent start")
            time.sleep(0.2)

    rospy.loginfo("Done")

    # aaaand we are done?




