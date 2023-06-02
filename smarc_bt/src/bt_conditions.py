#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


import math
import rospy
import py_trees as pt
import tf
import numpy as np

import bb_enums

from mission_plan import MissionPlan

from smarc_bt.msg import MissionControl

class C_TimeoutNotReached(pt.behaviour.Behaviour):
    """
    check if mission has reached timeout
    """
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        super(C_TimeoutNotReached, self).__init__(name="C_TimeoutNotReached")

    def update(self):
        plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if plan is None:
            self.feedback_message = "No plan for a timeout"
            return pt.Status.SUCCESS

        if plan.timeout_reached():
            self.feedback_message = "TIMEOUT"
            return pt.Status.FAILURE

        self.feedback_message = "{:.2f} remaining in mission".format(plan.time_remaining())
        return pt.Status.SUCCESS


class C_NoAbortReceived(pt.behaviour.Behaviour):
    """
    This condition returns FAILURE forever after it returns it once.
    Used as a one-time lock
    """
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.aborted = False
        super(C_NoAbortReceived, self).__init__(name="C_NoAbortReceived")

    def update(self):
        if self.bb.get(bb_enums.ABORT) or self.aborted:
            self.aborted = True
            self.feedback_message = 'ABORTED'
            return pt.Status.FAILURE

        plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if plan is not None:
            if plan.state == MissionControl.FB_EMERGENCY:
                self.feedback_message = 'EMERGENCY'
                self.aborted = True
                return pt.Status.FAILURE

        return pt.Status.SUCCESS


class C_LeakOK(pt.behaviour.Behaviour):
    def __init__(self):
        bb = pt.blackboard.Blackboard()
        self.vehicle = bb.get(bb_enums.VEHICLE_STATE)
        super(C_LeakOK, self).__init__(name="C_LeakOK")

    def update(self):
        if self.vehicle.leak == True:
            self.feedback_message = "\n\n\n!!!! LEAK !!!!\n\n\n"
            return pt.Status.FAILURE
        else:
            return pt.Status.SUCCESS


class C_DepthOK(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.max_depth = self.bb.get(bb_enums.MAX_DEPTH)
        super(C_DepthOK, self).__init__(name="C_DepthOK")


    def update(self):
        self.max_depth = self.bb.get(bb_enums.MAX_DEPTH)
        depth = self.vehicle.depth

        if depth is None:
            rospy.logwarn_throttle(5, "NO DEPTH READ! Success anyways")
            self.feedback_message = "Last read:None, max:{m:.2f}".format(l=depth, m=self.max_depth)
            return pt.Status.SUCCESS
        else:
            self.feedback_message = "Last read:{l:.2f}, max:{m:.2f}".format(l=depth, m=self.max_depth)

        if depth < self.max_depth:
            return pt.Status.SUCCESS
        else:
            rospy.logwarn_throttle(5, "Too deep!"+str(depth))
            return pt.Status.FAILURE



class C_AltOK(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        self.min_alt = self.bb.get(bb_enums.MIN_ALTITUDE)
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        super(C_AltOK, self).__init__(name="C_AltOK")

    def update(self):
        self.min_alt = self.bb.get(bb_enums.MIN_ALTITUDE)
        alt = self.vehicle.altitude
        if alt is None:
            rospy.logwarn_throttle(10, "NO ALTITUDE READ! The tree will run anyways")
            self.feedback_message = "Last read:None, min:{m:.2f}".format(m=self.min_alt)
            return pt.Status.SUCCESS
        else:
            self.feedback_message = "Last read:{l:.2f}, min:{m:.2f}".format(l=alt, m=self.min_alt)

        if alt > self.min_alt:
            return pt.Status.SUCCESS
        else:
            rospy.loginfo_throttle(5, "Too close to the bottom! "+str(alt))
            return pt.Status.FAILURE


class C_ExpectPlanState(pt.behaviour.Behaviour):
    def __init__(self, expected_state):
        """
        Return success if the mission plan has the given expected state
        """
        self.bb = pt.blackboard.Blackboard()
        s = MissionPlan.state_names[expected_state]
        super(C_ExpectPlanState, self).__init__(name="C_ExpectPlanState({})".format(s))
        self.expected_state = expected_state
        self.s = s

    def update(self):
        plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if plan is None:
            self.feedback_message = "No plan"
            return pt.Status.FAILURE

        if plan.state == self.expected_state:
            self.feedback_message = "is {}".format(self.s)
            return pt.Status.SUCCESS

        self.feedback_message = "Not {}".format(self.s)
        return pt.Status.FAILURE





