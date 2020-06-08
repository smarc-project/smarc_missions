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

from bt_common import CBFCondition


class C_AtDVLDepth(pt.behaviour.Behaviour):
    """
    Returns SUCCESS if at some specified depth
    """
    def __init__(self, dvl_depth):
        self.bb = pt.blackboard.Blackboard()
        self.dvl_depth = dvl_depth
        super(C_AtDVLDepth, self).__init__(name="C_AtDVLDepth")

    def update(self):
        depth = self.bb.get(bb_enums.DEPTH)
        if depth is None or depth < self.dvl_depth:
            rospy.loginfo_throttle(10, "Not deep enough for DVL: {}".format(depth))
            return pt.Status.FAILURE

        return pt.Status.SUCCESS



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

        if depth is None:
            rospy.logwarn_throttle(5, "NO DEPTH READ!")
            return pt.Status.SUCCESS

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

        self.cbf_condition = CBFCondition(checked_field_topic=None,
                                          checked_field_name='altitude',
                                          limit_type='>',
                                          limit_value=self.min_alt,
                                          update_func=self.update)
        self.update = self.cbf_condition.update

    def update(self):
        alt = self.bb.get(bb_enums.ALTITUDE)
        if alt is None:
            rospy.logwarn_throttle(5, "NO ALTITUDE READ! The tree will run anyways")
            return pt.Status.SUCCESS

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
        plan_is_go = self.bb.get(bb_enums.PLAN_IS_GO)
        if plan_is_go is None or plan_is_go == False:
            rospy.loginfo_throttle_identical(5, "Waiting for start plan")
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
            rospy.loginfo_throttle_identical(5, "Plan is not done")
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


class C_NoNewPOIDetected(pt.behaviour.Behaviour):
    """
    returns SUCCESS until there is a POI detected that is sufficiently further
    away than the last known one. or if its the first one.
    This distance is governed by new_pos_distance
    """
    def __init__(self, new_poi_distance):
        super(C_NoNewPOIDetected, self).__init__(name="C_NoNewPOIDetected")
        self.bb = pt.blackboard.Blackboard()
        self.new_poi_distance = new_poi_distance
        self._last_known_poi = None

    def update(self):
        poi = self.bb.get(bb_enums.POI_POINT_STAMPED)
        if poi is None:
            rospy.loginfo_throttle_identical(10,"No POI :(")
            return pt.Status.SUCCESS

        if self._last_known_poi is None:
            # a poi exists but we didnt know any beforehand, new poi!
            self._last_known_poi = poi
            rospy.logwarn_throttle_identical(10,"Our first POI!")
            return pt.Status.FAILURE

        # we knew a poi, there is a poi we see, far enough?
        xdiff = poi.point.x-self._last_known_poi.point.x
        ydiff = poi.point.y-self._last_known_poi.point.y
        zdiff = poi.point.z-self._last_known_poi.point.z
        dist = math.sqrt( xdiff**2 + ydiff**2 + zdiff**2 )

        # its far enough!
        if dist > self.new_poi_distance:
            self._last_known_poi = poi
            rospy.logwarn_throttle_identical(10,"A new POI that is far enough!"+str(dist))
            return pt.Status.FAILURE

        # aww, not far enough
        self._last_known_poi = poi
        rospy.loginfo_throttle_identical(10,"Probably the same POI as before...")
        return pt.Status.SUCCESS


class C_AutonomyDisabled(pt.behaviour.Behaviour):
    def __init__(self):
        super(C_AutonomyDisabled, self).__init__(name="C_AutonomyDisabled")
        self.bb = pt.blackboard.Blackboard()

    def update(self):
        enabled = self.bb.get(bb_enums.ENABLE_AUTONOMY)
        if enabled:
            return pt.Status.FAILURE

        return pt.Status.SUCCESS


class C_LeaderFollowerEnabled(pt.behaviour.Behaviour):
    def __init__(self, enable_leader_follower):
        super(C_LeaderFollowerEnabled, self).__init__(name="C_LeaderFollowerEnabled")
        self.bb = pt.blackboard.Blackboard()
        self.enable_leader_follower = enable_leader_follower

    def update(self):
        if self.enable_leader_follower:
            return pt.Status.SUCCESS

        return pt.Status.FAILURE


class C_LeaderExists(pt.behaviour.Behaviour):
    def __init__(self, base_link, leader_link):
        self.leader_link = leader_link
        self.base_link = base_link
        # strings might be ever so slightly different...
        if leader_link in base_link or base_link in leader_link:
            self.leader_is_self = True
        else:
            self.leader_is_self = False

        # assume not by default
        self.leader_exists = False

        self.bb = pt.blackboard.Blackboard()
        self.listener = tf.TransformListener()

        super(C_LeaderExists, self).__init__(name="C_LeaderExists")

    def setup(self, timeout):
        if self.leader_is_self:
            rospy.logwarn_throttle(3, "I am the leader!")
            return True

        try:
            rospy.loginfo_throttle(3, "Waiting for transform from {} to {}...".format(self.base_link, self.leader_link))
            self.listener.waitForTransform(self.base_link, self.leader_link, rospy.Time(), rospy.Duration(timeout))
            self.leader_exists = True
            rospy.loginfo_throttle(3, "...Got it, we got a leader to follow!")
        except:
            rospy.logwarn_throttle(5, "Could not find xform from {} to {}, assuming there is no leader!".format(self.base_link,self.leader_link))

        return True

    def update(self):
        if self.leader_is_self:
            rospy.logwarn_throttle(30, "I am the leader!")
            return pt.Status.FAILURE

        if not self.leader_exists:
            rospy.logwarn_throttle(60, "No leader!")
            return pt.Status.FAILURE

        return pt.Status.SUCCESS


class C_LeaderIsFarEnough(pt.behaviour.Behaviour):
    def __init__(self, base_link, leader_link, min_distance_to_leader):
        self.leader_link = leader_link
        self.base_link = base_link
        self.min_distance_to_leader = min_distance_to_leader
        self.bb = pt.blackboard.Blackboard()
        self.listener = tf.TransformListener()
        self.leader_exists = False
        super(C_LeaderIsFarEnough, self).__init__(name="C_LeaderIsFarEnough")


    def setup(self, timeout):
        try:
            rospy.loginfo_throttle(3, "Waiting for transform from {} to {}...".format(self.base_link, self.leader_link))
            self.listener.waitForTransform(self.base_link, self.leader_link, rospy.Time(), rospy.Duration(timeout))
            self.leader_exists = True
            rospy.loginfo_throttle(3, "...Got it, we got a leader to follow!")
        except:
            rospy.logwarn_throttle(5, "Could not find xform from {} to {}, assuming there is no leader!".format(self.base_link,self.leader_link))

        return True


    def update(self):
        if not self.leader_exists:
            return pt.Status.FAILURE

        trans, rot = self.listener.lookupTransform(self.base_link,
                                                   self.leader_link,
                                                   rospy.Time(0))
        dist = np.linalg.norm(trans)
        if dist > self.min_distance_to_leader:
            return pt.Status.SUCCESS

        rospy.loginfo_throttle(5, "Leader {} is too close to me {} !".format(self.leader_link, self.base_link))
        return pt.Status.FAILURE



