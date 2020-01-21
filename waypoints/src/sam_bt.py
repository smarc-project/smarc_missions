#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import py_trees as pt, py_trees_ros as ptr, rospy, json
from behaviours import Sequence
import pt.composites.Selector as Fallback # just convenience really

from bt_actions import A_SetMissionPlan, \
                       A_PublishToNeptus, \
                       A_ExecutePlanAction, \
                       A_SetManualWaypoint, \
                       A_GotoManualWaypoint, \
                       A_SetNextPlanAction

from bt_conditions import C_HaveMission, \
                          C_PlanCompleted, \
                          C_NoAbortReceived, \
                          C_HaveManualWaypoint, \
                          C_ManualWaypointReceived, \
                          C_NewMissionPlanReceived

def const_feedback_tree():
    neptus_feedback = A_PublishToNeptus()
    # more feedback options will go here

    return Sequence(name="SQ-Feedback",
                    children=[
                              neptus_feedback
                    ])



def const_safety_tree():
    no_abort = C_NoAbortReceived()
    # more safety checks will go here

    return Sequence(name="SQ-Safety",
                    children=[
                              no_abort
                    ])


def const_synch_tree():
    def const_check_manual_commands():
        def const_check_manual_waypoint():
            check_received = C_ManualWaypointReceived()
            set_manual_wp = A_SetManualWaypoint()
            return Sequence(name="SQ-CheckManualWaypoint"
                            children=[
                                      check_received,
                                      set_manual_wp
                                     ])

        manual_wps = const_check_manual_waypoint()
        # More manual commands that are not waypoints
        # will go here

        return Sequence(name="SQ-CheckManualCommands",
                        children=[
                                  manual_wps
                        ])

    def const_mission_plan_update():
        got_new_plan = C_NewMissionPlanReceived()
        set_new_plan = A_SetMissionPlan()
        return Sequence(name="SQ-UpdateMissionPlan",
                        children=[
                                  got_new_plan,
                                  set_new_plan
                        ])


    manual_commands = const_check_manual_commands()
    mission_plan = const_mission_plan_update()
    # more synchronization actions can go here

    # in the end, we want to succeed somehow.
    have_mission = C_HaveMission()

    return Fallback(name="FB-SynchroniseMission",
                    children=[
                              manual_commands,
                              mission_plan,
                              have_mission
                    ])


def const_execute_mission_tree():
    def const_execute_manual_commands():
        def const_gotomanualwp():
            have_wp = C_HaveManualWaypoint()
            goto_manual_wp = A_GotoManualWaypoint()
            return Sequence(name="SQ-GotoManualWaypoint",
                            children=[
                                      have_wp,
                                      goto_manual_wp
                            ])
        # def const_enable/disable tihngs etc commands
        goto_manual = const_gotomanualwp()
        # add more manual commands here

        return Fallback(name="FB-ExecuteManualComamands",
                        children=[
                                  goto_manual
                        ])

    def const_execute_mission_plan():
        plan_complete = C_PlanCompleted()
        execute_plan_action = A_ExecutePlanAction()
        set_next_plan_action = A_SetNextPlanAction()

        follow_plan = Sequence(name="SQ-FollowMissionPlan",
                               children=[
                                         execute_plan_action,
                                         set_next_plan_action
                               ])

        return Fallback(name="FB-ExecuteMissionPlan",
                        children=[
                                  plan_complete,
                                  follow_plan
                        ])


    manual_commands = const_execute_manual_commands()
    mission_plan = const_execute_mission_plan()
    # add more mission actions here

    return Fallback(name="FB-ExecuteMission",
                    children=[
                              manual_commands,
                              mission_plan
                    ])


def const_finalize_mission_tree():

    idle = pt.behaviours.Running(name="Idle")

    return Sequence(name="SQ-FinalizeMission",
                    children=[
                              idle
                    ])




def const_tree():
    feedback_tree = const_feedback_tree()
    safety_tree = const_safety_tree()
    synch_mission_tree = const_synch_tree()
    exec_mission_tree = const_execute_mission_tree()
    finalize_mission_tree = const_finalize_mission_tree()

    return Sequence(name='SQ-ROOT',
                    children=[
                              feedback_tree,
                              safety_tree,
                              synch_mission_tree,
                              exec_mission_tree,
                              finalize_mission_tree])

