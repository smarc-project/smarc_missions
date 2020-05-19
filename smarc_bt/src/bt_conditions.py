#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


import rospy
import py_trees as pt
import bb_enums
import imc_enums


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
        FAILURE otherwise.

        both returns will latch until the opposite is received since the listener will
        only overwrite the previous message when a new one is received.

        returns FAILURE if no message received.
        """
        self.bb = pt.blackboard.Blackboard()
        super(C_StartPlanReceived, self).__init__(name="C_StartPlanReceived")

    def update(self):
        plan_control_msg = self.bb.get(bb_enums.PLAN_CONTROL_MSG)
        if plan_control_msg is None:
            return pt.Status.FAILURE

        # check if this message is a 'go' or 'no go' message
        # imc/plan_control(569):
        # int type:[0,1,2,3] req,suc,fail,in prog
        # int op:[0,1,2,3] start, stop, load, get
        # int request_id
        # string plan_id
        # int flags
        # string info

        # the start button in neptus sends:
        # type:0 op:0 plan_id:"string" flags:1
        # stop button sends:
        # type:0 op:1 plan_id:'' flags:1
        # teleop button sends:
        # type:0 op:0 plan_id:"teleoperation-mode" flags:0

        typee = plan_control_msg.type
        op = plan_control_msg.op
        plan_id = plan_control_msg.plan_id
        flags = plan_control_msg.flags

        # separate well-defined ifs for possible future shenanigans.
        if typee==0 and op==0 and plan_id!='' and flags==1:
            # start button

            # check if the start was given for our current plan
            current_mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
            if current_mission_plan is not None and plan_id == current_mission_plan.plan_id:
                rospy.loginfo_throttle_identical(20, "Started plan:"+str(plan_id))
                return pt.Status.SUCCESS
            else:
                rospy.logwarn_throttle_identical(10, "Start was given for a different plan than our plan, so we wont start it!")
                rospy.logwarn_throttle_identical(10, "Start given for plan:"+str(plan_id)+" our plan:"+str(current_mission_plan.plan_id))
                return pt.Status.FAILURE

        if typee==0 and op==1 and plan_id=='' and flags==1:
            # stop button
            return pt.Status.FAILURE

        # this string is hardcoded in Neptus, so we hardcode it here too!
        if typee==0 and op==1 and plan_id=='teleoperation-mode' and flag==0:
            # teleop button
            return pt.Status.FAILURE



class C_NewMissionPlanReceived(pt.behaviour.Behaviour):
    def __init__(self):
        """
        returns SUCCESS if there is a new and different
        mission plan received.
        Does not attempt to parse the mission plan and simply stores it in
        MISSION_PLAN_STR.

        return FAILURE otherwise.
        """

        self.bb = pt.blackboard.Blackboard()
        self.last_known_plan_msg=''
        super(C_NewMissionPlanReceived, self).__init__(name="C_NewMissionPlanReceived")


    def update(self):
        current_plan_msg = self.bb.get(bb_enums.MISSION_PLAN_MSG)

        # a bad message or a duplicate
        if current_plan_msg is None or\
           current_plan_msg == self.last_known_plan_msg:
            return pt.Status.FAILURE

        # we ignore other types of plan operations for now.
        if current_plan_msg.op != imc_enums.PLANDB_OP_SET:
            return pt.Status.FAILURE

        # all is well, let the tree read it.
        self.last_known_plan_msg = current_plan_msg
        self.logger.info("New mission plan received:"+str(current_plan_msg))
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
        if mission_plan is None or not mission_plan.completed:
            return pt.Status.FAILURE

        return pt.Status.SUCCESS


