#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import py_trees as pt, py_trees_ros as ptr
# just convenience really
from py_trees.composites import Selector as Fallback
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

import rospy

from bt_actions import A_SetMissionPlan, \
                       A_PublishToNeptus, \
                       A_ExecutePlanAction, \
                       A_SetManualWaypoint, \
                       A_GotoManualWaypoint, \
                       A_SetNextPlanAction, \
                       A_UpdateTF


from bt_conditions import C_PlanCompleted, \
                          C_HaveManualWaypoint, \
                          C_ManualWaypointReceived, \
                          C_NewMissionPlanReceived, \
                          C_DepthOK
                          #  C_AltOK, \ # unused for now

from bt_common import Sequence

from sam_globals import *

def const_tree():
    """
    construct the entire tree.
    the structure of the code reflects the structure of the tree itself.
    sub-trees are constructed in inner functions.
    """

    def const_data_ingestion_tree():
        read_depth = ptr.subscribers.ToBlackboard(
            name = "A_ReadDepth",
            topic_name = DEPTH_TOPIC,
            topic_type = Float64,
            blackboard_variables = {DEPTH_BB:'data'} # this takes the Float64.data field and puts into the bb
        )
        #read_alt = same as depth eventually goes here
        #  update_tf = A_UpdateTF()

        return Sequence(name="SQ-DataIngestion",
                        children=[
                            read_depth
                            #  update_tf
                        ])



    def const_feedback_tree():
        neptus_feedback = A_PublishToNeptus()
        # more feedback options will go here

        return Sequence(name="SQ-Feedback",
                        children=[
                                  neptus_feedback
                        ])



    def const_safety_tree():
        #  no_abort = C_NoAbortReceived()
        # TODO does not exist in our version of py_trees!
        no_abort = pt.behaviours.CheckBlackboardVariableValue(
            variable_name=ABORT_BB,
            expected_value=False,
            name="C_NoAbortReceived"
        )
        #TODO re-enable when the topic is there
        #  altOK = C_AltOK()
        depthOK = C_DepthOK()
        # more safety checks will go here

        return Sequence(name="SQ-Safety",
                        children=[
                                  no_abort,
                                  #  altOK,
                                  depthOK
                        ])


    def const_synch_tree():
        def const_check_manual_commands():
            def const_check_manual_waypoint():
                check_received = C_ManualWaypointReceived()
                set_manual_wp = A_SetManualWaypoint()
                return Sequence(name="SQ-CheckManualWaypoint",
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
        have_mission = pt.blackboard.CheckBlackboardVariable(name="C_HaveMission",
                                                             variable_name=MISSION_PLAN_OBJ)

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


    data_ingestion_tree = const_data_ingestion_tree()
    feedback_tree = const_feedback_tree()
    safety_tree = const_safety_tree()
    synch_mission_tree = const_synch_tree()
    exec_mission_tree = const_execute_mission_tree()
    finalize_mission_tree = const_finalize_mission_tree()

    root = Sequence(name='SQ-ROOT',
                    children=[
                              feedback_tree,
                              safety_tree,
                              synch_mission_tree,
                              exec_mission_tree,
                              finalize_mission_tree])

    return ptr.trees.BehaviourTree(root)



def main():

    utm_zone = rospy.get_param("utm_zone")
    utm_band = rospy.get_param("utm_band")

    bb = pt.blackboard.Blackboard()
    bb.set(UTM_ZONE_BB, utm_zone)
    bb.set(UTM_BAND_BB, utm_band)


    try:
        rospy.loginfo("Constructing tree")
        tree = const_tree()
        rospy.loginfo("Setting up tree")
        tree.setup(timeout=10)
        rospy.loginfo("Ticktocking....")
        while not rospy.is_shutdown():
            # rate is period in ms
            #  tree.tick_tock(1, post_tick_handler=lambda t: pt.display.print_ascii_tree(tree.root, show_status=True))
            tree.tick_tock(100)

    except rospy.ROSInitException:
        rospy.loginfo("ROS Interrupt")


if __name__ == '__main__':
    # init the node
    rospy.init_node("sam_bt")

    #  main()
    tree = const_tree()
    pt.display.ascii_tree(tree.root)

