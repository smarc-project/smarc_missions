#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


import math
import rospy
import py_trees as pt
import tf
import numpy as np

import imc_enums
import bb_enums

class C_CheckWaypointType(pt.behaviour.Behaviour):
    """
    Checks if the type of the current WP corresponds to the given
    expected type and returns SUCCESS if they match

    Use the imc_enums.MANEUVER_XXX as the expected wp type
    """
    def __init__(self,
                 expected_wp_type,
                 bb_key = None):
        self.bb = pt.blackboard.Blackboard()
        self.expected_wp_type = expected_wp_type
        self.expected_wp_type_str = C_CheckWaypointType.imc_id_to_str(self.expected_wp_type)

        self.bb_key = bb_key

        super(C_CheckWaypointType, self).__init__(name="C_CheckWaypointType = {}".format(self.expected_wp_type_str))


    @staticmethod
    def imc_id_to_str(imc_id):
        if imc_id == imc_enums.MANEUVER_GOTO:
            return imc_enums.MANEUVER_GOTO_STR
        if imc_id == imc_enums.MANEUVER_SAMPLE:
            return imc_enums.MANEUVER_SAMPLE_STR

        return str(imc_id)

    def update(self):
        self.feedback_message = "Got None"

        if self.bb_key is None:
            mission = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
            if mission is None:
                return pt.Status.FAILURE


            wp = mission.get_current_wp()
        else:
            wp = self.bb.get(self.bb_key)

        if wp is None:
            return pt.Status.FAILURE

        self.feedback_message = "Got:{}".format(C_CheckWaypointType.imc_id_to_str(wp.imc_man_id))
        if wp.imc_man_id != self.expected_wp_type:
            return pt.Status.FAILURE

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
        if self.bb.get(bb_enums.ABORT) or self.aborted or self.vehicle.aborted:
            self.aborted = True
            self.vehicle.abort()
            self.feedback_message = 'ABORTED'
            return pt.Status.FAILURE
        else:
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


class C_StartPlanReceived(pt.behaviour.Behaviour):
    def __init__(self):
        """
        return SUCCESS if we the tree received a plan_control message that
        said 'run plan'.
        Currently being set bt A_UpdateNeptusPlanControl.
        FAILURE otherwise.
        """
        self.bb = pt.blackboard.Blackboard()
        super(C_StartPlanReceived, self).__init__(name="C_StartPlanReceived")

    def update(self):
        plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if plan is None:
            self.feedback_message = "No plan"
            return pt.Status.FAILURE

        self.feedback_message = "Plan is go:{} for plan {}".format(plan.plan_is_go, plan.plan_id)
        if plan.plan_is_go:
            return pt.Status.SUCCESS
        else:
            rospy.loginfo_throttle_identical(5, "Waiting for start plan")
            return pt.Status.FAILURE


class C_PlanCompleted(pt.behaviour.Behaviour):
    def __init__(self):
        """
        If the currently know MissionPlan object in MISSION_PLAN_OBJ
        has no more actions left to do, return SUCCESS

        return FAILURE otherwise
        """
        self.bb = pt.blackboard.Blackboard()
        super(C_PlanCompleted, self).__init__(name="C_PlanCompleted")

    def update(self):
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            msg = "No plan received yet"
            self.feedback_message = msg
            rospy.loginfo_throttle(5, msg)
            return pt.Status.FAILURE
        elif not mission_plan.is_complete():
            msg = "Progress:{}/{} on plan {}".format(mission_plan.current_wp_index, len(mission_plan.waypoints), mission_plan.plan_id)
            self.feedback_message = msg
            rospy.loginfo_throttle_identical(5, msg)
            return pt.Status.FAILURE

        rospy.loginfo_throttle_identical(2, "Plan is complete!")
        self.feedback_message = "Current plan:{}".format(mission_plan.plan_id)
        return pt.Status.SUCCESS


class C_HaveCoarseMission(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        super(C_HaveCoarseMission, self).__init__(name="C_HaveCoarseMission")

    def update(self):
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None or mission_plan.waypoints is None or len(mission_plan.waypoints) <= 0:
            return pt.Status.FAILURE

        self.feedback_message = "Current plan:{}".format(mission_plan.plan_id)
        return pt.Status.SUCCESS

class C_PlanIsNotChanged(pt.behaviour.Behaviour):
    """
    Use this condition to stop a running action when a plan with a different
    plan_id or bigger time stamp is seen in the tree.
    """
    def __init__(self):
        super(C_PlanIsNotChanged, self).__init__(name="C_PlanIsNotChanged")
        self.bb = pt.blackboard.Blackboard()
        self.last_known_id = None
        self.last_known_time = 0

    def update(self):
        current_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_plan is None:
            # there is no plan, it can not change
            self.last_known_id = None
            self.last_known_time = 0
            self.feedback_message = "There was no plan, plan is not changed"
            rospy.loginfo_throttle_identical(10, self.feedback_message)
            return pt.Status.SUCCESS

        if self.last_known_id is None:
            # this is the first plan we saw
            # record it, and let the tree tick again
            self.last_known_id = current_plan.plan_id
            self.last_known_time = current_plan.creation_time
            self.feedback_message = "First time seeing any plan, plan is changed"
            rospy.loginfo_throttle_identical(10, self.feedback_message)
            return pt.Status.FAILURE

        if self.last_known_id == current_plan.plan_id and self.last_known_time < current_plan.creation_time:
            # this is the same plan, but it was sent again, this means a restart
            # so the plan IS changed
            self.feedback_message = "Same plan_id, but the current plan is newer, plan is changed"
            rospy.loginfo_throttle_identical(10, self.feedback_message)
            self.last_known_id = current_plan.plan_id
            self.last_known_time = current_plan.creation_time
            return pt.Status.FAILURE

        if self.last_known_id != current_plan.plan_id:
            # the plan has been changed completely
            self.feedback_message = "A new plan is received!"
            rospy.loginfo_throttle_identical(10, self.feedback_message)
            self.last_known_id = current_plan.plan_id
            self.last_known_time = current_plan.creation_time
            current_plan.plan_is_go = False
            return pt.Status.FAILURE

        self.feedback_message = "last_id:{}, current_id:{}".format(self.last_known_id, current_plan.plan_id)
        rospy.loginfo_throttle_identical(10, self.feedback_message)
        return pt.Status.SUCCESS



