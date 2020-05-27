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
        if self.bb.get(bb_enums.DEPTH) < self.max_depth:
            return pt.Status.SUCCESS
        else:
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
        # remove this when there is altitude available
        return pt.Status.SUCCESS

        if self.bb.get(bb_enums.ALTITUDE) > self.min_alt:
            return pt.Status.SUCCESS
        else:
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


