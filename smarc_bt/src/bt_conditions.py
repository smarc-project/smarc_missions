#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


import rospy
import py_trees as pt
import bb_enums
import imc_enums

from bt_common import CBFCondition


class C_NoAbortReceived(pt.behaviour.Behaviour):
    """
    This condition returns FAILURE forever after it returns it once.
    Used as a one-time lock
    """
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        self.aborted = False
        super(C_NoAbortReceived, self).__init__(name="C_NoAbortReceived")

    def update(self):
        if self.bb.get(bb_enums.ABORT) or self.aborted:
            self.aborted = True
            return pt.Status.FAILURE
        else:
            return pt.Status.SUCCESS

class C_DepthOK(pt.behaviour.Behaviour):
    def __init__(self, max_depth):
        self.bb = pt.blackboard.Blackboard()
        self.max_depth = max_depth
        super(C_DepthOK, self).__init__(name="C_DepthOK")

        self.cbf_condition = CBFCondition(checked_field_topic=None,
                                          checked_field_name='depth',
                                          limit_type='<',
                                          limit_value=self.max_depth,
                                          update_func=self.update)
        self.update = self.cbf_condition.update

    def update(self):
        depth = self.bb.get(bb_enums.DEPTH)
        if depth < self.max_depth:
            return pt.Status.SUCCESS
        else:
            rospy.logwarn_throttle(5, "Too deep!"+str(depth))
            return pt.Status.FAILURE


class C_LeakOK(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        super(C_LeakOK, self).__init__(name="C_LeakOK")

    def update(self):
        if self.bb.get(bb_enums.LEAK) == True:
            return pt.Status.FAILURE
        else:
            return pt.Status.SUCCESS



class C_AltOK(pt.behaviour.Behaviour):
    def __init__(self, min_alt):
        self.bb = pt.blackboard.Blackboard()
        self.min_alt = min_alt
        super(C_AltOK, self).__init__(name="C_AltOK")

    def update(self):
        alt = self.bb.get(bb_enums.ALTITUDE)
        if alt > self.min_alt:
            return pt.Status.SUCCESS
        else:
            rospy.logwarn_throttle(5, "Too close to the bottom!"+str(alt))
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
        #TODO probably can be done with a built-in class?
        plan_is_go = self.bb.get(bb_enums.PLAN_IS_GO)
        if plan_is_go is None or plan_is_go == False:
            return pt.Status.FAILURE
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
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None or not mission_plan.is_complete():
            return pt.Status.FAILURE

        return pt.Status.SUCCESS

class C_HaveRefinedMission(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        super(C_HaveRefinedMission, self).__init__(name="C_HaveRefinedMission")

    def update(self):
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None or mission_plan.refined_waypoints is None:
            return pt.Status.FAILURE

        return pt.Status.SUCCESS

class C_HaveCoarseMission(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        super(C_HaveCoarseMission, self).__init__(name="C_HaveCoarseMission")

    def update(self):
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None or mission_plan.waypoints is None:
            return pt.Status.FAILURE

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
            rospy.loginfo_throttle_identical(10, "There was no plan, plan is not changed")
            return pt.Status.SUCCESS

        if self.last_known_id is None:
            # this is the first plan we saw
            # record it, and let the tree tick again
            self.last_known_id = current_plan.plan_id
            self.last_known_time = current_plan.creation_time
            rospy.loginfo_throttle_identical(10, "First time seeing any plan, plan is changed")
            return pt.Status.FAILURE

        if self.last_known_id == current_plan.plan_id and self.last_known_time < current_plan.creation_time:
            # this is the same plan, but it was sent again, this means a restart
            # so the plan IS changed
            rospy.loginfo_throttle_identical(10, "Same plan_id, but the current plan is newer, plan is changed")
            self.last_known_id = current_plan.plan_id
            self.last_known_time = current_plan.creation_time
            return pt.Status.FAILURE

        rospy.loginfo_throttle_identical(60, "Plan is not changed")
        return pt.Status.SUCCESS



