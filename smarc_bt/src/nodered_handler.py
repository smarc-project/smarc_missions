#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

import rospy, time
import numpy as np

from mission_plan import MissionPlan
import imc_enums, bb_enums

from smarc_msgs.msg import MissionControl

class NoderedHandler(object):
    """
    A parallel to the neptus handler, that handles the
    communication with the nodered interface
    this includes things like start/stop/pause plan
    asnwer questions about the plan and set up mission plans
    """
    def __init__(self,
                 auv_config,
                 vehicle,
                 blackboard):

        self._vehicle = vehicle
        self._config = auv_config
        self._bb = blackboard

        self._last_received_mc_msg = None
        self._mission_control_sub = rospy.Subscriber(self._config.MISSION_CONTROL_TOPIC,
                                                     MissionControl,
                                                     self._mission_control_cb,
                                                     queue_size=1)

        self._mc_msg = MissionControl()
        self._mission_control_pub = rospy.Publisher(self._config.MISSION_CONTROL_TOPIC,
                                                    MissionControl,
                                                    queue_size=1)

    def _mission_control_cb(self, msg):
        self._last_received_mc_msg = msg

    def _publish_current_plan(self):
        # simply publish the current plan at every tick
        self._mc_msg.command = MissionControl.CMD_IS_FEEDBACK
        mission_plan = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            self._mc_msg.name = "No plan"
            self._mc_msg.plan_state = MissionControl.FB_STOPPED
            self._mc_msg.waypoints = []
        else:
            # there is a plan, inform the planner of its state
            self._mc_msg.name = mission_plan.plan_id
            if mission_plan.is_complete():
                self._mc_msg.plan_state = MissionControl.FB_STOPPED
            elif mission_plan.is_in_progress():
                if  mission_plan.plan_is_go:
                    self._mc_msg.plan_state = MissionControl.FB_RUNNING
                else:
                    self._mc_msg.plan_state = MissionControl.FB_PAUSED
            else:
                self._mc_msg.plan_state = MissionControl.FB_RECEIVED

            # XXX maybe check emergency as well, maybe not meh

            # mission plan should contain a list of waypoint objects that each contain
            # a GotoWaypoint object called wp
            self._mc_msg.waypoints = [wp.wp for wp in mission_plan.waypoints]

        self._mission_control_pub.publish(self._mc_msg)

    def tick(self):
        self._publish_current_plan()

        if self._last_received_mc_msg is None:
            return





