#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


#TODO
# . C_HaveManualWaypoint
# . C_ManualWaypointReceived


from bt_common import *
import rospy
from std_msgs.msg import Empty, String, Float64

class C_NoAbortReceived(ptr.subscribers.Handler):
    def __init__(self):
        """
        copied from Chris, simplified
        Returns success as long as there is not
        any message recieved at /abort.
        """
        # become a behaviour
        super(C_NoAbortReceived, self).__init__(
            name="C_NoAbortReceived",
            topic_name=ABORT_TOPIC,
            topic_type=Empty,
            clearing_policy=pt.common.ClearingPolicy.ON_SUCCESS
        )

    def update(self):
        with self.data_guard:
            # do not abort
            if self.msg is None:
                return pt.Status.SUCCESS
            # abort
            else:
                return pt.Status.FAILURE


class C_DepthOK(ptr.subscribers.Handler):
    def __init__(self):
        # become a behaviour
        super(C_DepthOK, self).__init__(
            name="C_DepthOK",
            topic_name=DEPTH_TOPIC,
            topic_type=Float64,
            clearing_policy=pt.common.ClearingPolicy.ON_SUCCESS
        )

    def update(self):
        with self.data_guard:
            # do not abort
            if self.msg is not None:
                if self.msg.data < SAM_MAX_DEPTH:
                    return pt.Status.SUCCESS
                else:
                    return pt.Status.FAILURE


class C_AltOK(ptr.subscribers.Handler):
    def __init__(self):
        # become a behaviour
        super(C_AltOK, self).__init__(
            name="C_AltOK",
            topic_name=ALTITUDE_TOPIC,
            topic_type=Float64,
            clearing_policy=pt.common.ClearingPolicy.ON_SUCCESS
        )

    def update(self):
        with self.data_guard:
            # do not abort
            if self.msg is not None:
                if self.msg.data > SAM_MIN_ALTITUDE:
                    return pt.Status.SUCCESS
                else:
                    return pt.Status.FAILURE



class C_NewMissionPlanReceived(ptr.subscribers.Handler):
    def __init__(self):
        """
        returns SUCCESS if there is a new and different
        mission plan received from Neptus.
        Does not attempt to parse the mission plan and simply stores it in
        MISSION_PLAN_STR.

        return FAILURE otherwise.
        """

        self.bb = pt.blackboard.Blackboard()

        # become a behaviour
        super(C_NewMissionPlanReceived, self).__init__(
            name="C_NewMissionPlanReceived?",
            topic_name=PLAN_TOPIC,
            topic_type=String,
            clearing_policy=pt.common.ClearingPolicy.ON_SUCCESS
        )

    def update(self):
        with self.data_guard:
            # nothing received, or its too small to be useful
            if self.msg is None or len(str(self.msg.data)) < MINIMUM_PLAN_STR_LEN:
                return pt.Status.FAILURE

            else:
                self.bb.set(MISSION_PLAN_STR, self.msg.data)
                rospy.loginfo("Set MISSION_PLAN_STR to"+str(self.msg.data))
                return pt.Status.SUCCESS



class C_PlanCompleted(pt.behaviour.Behaviour):
    def __init__(self):
        """
        If the currently know MissionPlan object in MISSION_PLAN_OBJ
        has no more actions left to do, return SUCCESS

        return FAILURE otherwise
        """
        self.bb = pt.blackboard.Blackboard()
        super(C_PlanCompleted, self).__init__(name="C_PlanCompleted?")

    def update(self):
        mission_plan = self.bb.get(MISSION_PLAN_OBJ)
        if mission_plan is None or len(mission_plan.remaining_wps) > 0:
            return pt.Status.FAILURE

        return pt.Status.SUCCESS


##########################################################################################
# NOT IMPLEMENTED YET
##########################################################################################

class C_HaveManualWaypoint(pt.behaviour.Behaviour):
    def __init__(self):
        #TODO implement to allow for manual single waypoints to be
        # processed by the tree, without sending a whole new plan

        self.bb = pt.blackboard.Blackboard()
        super(C_HaveManualWaypoint, self).__init__("C_HaveManualWaypoint")

    def update(self):
        return pt.Status.FAILURE


class C_ManualWaypointReceived(pt.behaviour.Behaviour):
    def __init__(self):
        #TODO implement to allow for manual single waypoints to be
        # processed by the tree, without sending a whole new plan

        self.bb = pt.blackboard.Blackboard()
        super(C_ManualWaypointReceived, self).__init__("C_ManualWaypointReceived")

    def update(self):
        return pt.Status.FAILURE
