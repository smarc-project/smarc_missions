#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8



#TODO
# . C_HaveManualWaypoint
# . C_ManualWaypointReceived


from bt_common import *
import py_trees as pt

from sam_globals import *
import sam_globals


class C_NoAbortReceived(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        self.aborted = False
        super(C_NoAbortReceived, self).__init__(name="C_NoAbortReceived")

    def update(self):
        if self.bb.get(ABORT_BB) or self.aborted:
            self.aborted = True
            return pt.Status.FAILURE
        else:
            return pt.Status.SUCCESS

class C_DepthOK(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        super(C_DepthOK, self).__init__(name="C_DepthOK")

    def update(self):
        if self.bb.get(DEPTH_BB) < SAM_MAX_DEPTH:
            return pt.Status.SUCCESS
        else:
            return pt.Status.FAILURE


class C_LeakOK(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        super(C_LeakOK, self).__init__(name="C_LeakOK")

    def update(self):
        if self.bb.get(LEAK_BB) == True:
            return pt.Status.FAILURE
        else:
            return pt.Status.SUCCESS



# currently unused because sam doesnt measure altitude yet
class C_AltOK(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()
        super(C_AltOK, self).__init__(name="C_AltOK")

    def update(self):
        # remove this when there is altitude available
        return pt.Status.SUCCESS

        if self.bb.get(ALTITUDE_BB) > SAM_MIN_ALTITUDE:
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
        plan_control_msg = self.bb.get(PLAN_CONTROL_MSG_BB)
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
            return pt.Status.SUCCESS

        if typee==0 and op==1 and plan_id=='' and flags==1:
            # stop button
            return pt.Status.FAILURE

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
        current_plan_msg = self.bb.get(MISSION_PLAN_MSG_BB)

        # a bad message or a duplicate
        if current_plan_msg is None or\
           current_plan_msg == self.last_known_plan_msg:
            return pt.Status.FAILURE

        # we ignore other types of plan operations for now.
        if current_plan_msg.op != IMC_PLANDB_OP_SET:
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
        mission_plan = self.bb.get(MISSION_PLAN_OBJ_BB)
        if mission_plan is None or not mission_plan.completed:
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
