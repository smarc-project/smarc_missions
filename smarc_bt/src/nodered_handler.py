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

    def _command_matches_known_mission(self, msg):
        current_mission = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_mission is None:
            rospy.loginfo("Mission start/stop/pause given but there is no mission?")
            return False
        if current_mission.plan_id != msg.name:
            s = "Command given for: {}, but we got: {}, stopping current mission!".format(msg.name, current_mission.plan_id)
            rospy.loginfo(s)
            current_mission.plan_is_go = False
            return False
        return True

    def tick(self):
        self._publish_current_plan()

        if self._last_received_mc_msg is None:
            return

        # there might be a command from nodered, check it

        msg = self._last_received_mc_msg
        current_mission = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        if msg.command == MissionControl.CMD_START:
            # start a mission, but check that the start is given
            # for the mission we got
            # finally, start given for the mission we got...
            if self._command_matches_known_mission(msg):
                if current_mission.is_complete():
                    current_mission.current_wp_index = 0
                    current_mission.plan_is_go = True
                    rospy.loginfo("RE-Started mission {}".format(msg.name))
                else:
                    rospy.loginfo("Started mission {}".format(msg.name))
                    current_mission.plan_is_go = True

        elif msg.command == MissionControl.CMD_STOP:
            if self._command_matches_known_mission(msg):
                rospy.loginfo("Stopped and removed mission {}".format(msg.name))
                self._bb.set(bb_enums.MISSION_PLAN_OBJ, None)

        elif msg.command == MissionControl.CMD_PAUSE:
            if self._command_matches_known_mission(msg):
                rospy.loginfo("Paused mission {}".format(msg.name))
                current_mission.plan_is_go = False

        elif msg.command == MissionControl.CMD_EMERGENCY:
            self._vehicle.abort()
            rospy.logwarn("Aborted")

        elif msg.command == MissionControl.CMD_SET_PLAN:
            new_plan = MissionPlan(auv_config = self._config,
                                   plan_id = msg.name,
                                   mission_control_msg = msg,
                                   coverage_swath = self._bb.get(bb_enums.SWATH),
                                   vehicle_localization_error_growth = self._bb.get(bb_enums.LOCALIZATION_ERROR_GROWTH))
            self._bb.set(bb_enums.MISSION_PLAN_OBJ, new_plan)
            rospy.loginfo("New mission {} set!".format(msg.name))

        elif msg.command == MissionControl.CMD_IS_FEEDBACK:
            # do nothing with feedback
            pass

        elif msg.command == MissionControl.CMD_REQUEST_FEEDBACK:
            self._publish_current_plan()

        else:
            pass






