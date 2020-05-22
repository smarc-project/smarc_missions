#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import py_trees as pt
import py_trees_ros as ptr

# just convenience really
from py_trees.composites import Selector as Fallback
from std_msgs.msg import Float64

from sam_msgs.msg import Leak
from imc_ros_bridge.msg import PlanDB, PlanControl

import rospy

from bt_actions import A_SetMissionPlan, \
                       A_PublishToNeptus, \
                       A_ExecutePlanAction, \
                       A_SetNextPlanAction, \
                       A_UpdateTF, \
                       A_EmergencySurface, \
                       A_AnswerNeptusPlanReceived, \
                       A_SetUTMFromGPS


from bt_conditions import C_PlanCompleted, \
                          C_NewMissionPlanReceived, \
                          C_NoAbortReceived, \
                          C_DepthOK, \
                          C_AltOK, \
                          C_LeakOK, \
                          C_StartPlanReceived

from bt_common import Sequence, \
                      CheckBlackboardVariableValue, \
                      ReadTopic

import bb_enums
import imc_enums

import common_globals

def const_tree(auv_config):
    """
    construct the entire tree.
    the structure of the code reflects the structure of the tree itself.
    sub-trees are constructed in inner functions.
    auv_config is in scope of all these inner functions.

    auv_config is a simple data object with a bunch of UPPERCASE fields in it.
    """

    def const_data_ingestion_tree():
        read_abort = ptr.subscribers.EventToBlackboard(
            name = "A_ReadAbort",
            topic_name = auv_config.ABORT_TOPIC,
            variable_name = bb_enums.ABORT
        )

        read_depth = ReadTopic(
            name = "A_ReadDepth",
            topic_name = auv_config.DEPTH_TOPIC,
            topic_type = Float64,
            blackboard_variables = {bb_enums.DEPTH:'data'} # this takes the Float64.data field and puts into the bb
        )

        read_alt = ReadTopic(
            name = "A_ReadAlt",
            topic_name = auv_config.ALTITUDE_TOPIC,
            topic_type = Float64,
            blackboard_variables = {bb_enums.ALTITUDE:'data'} # this takes the Float64.data field and puts into the bb
        )

        read_leak = ReadTopic(
            name = "A_ReadLeak",
            topic_name = auv_config.LEAK_TOPIC,
            topic_type = Leak,
            blackboard_variables = {bb_enums.LEAK:'value'}
        )


        update_tf = A_UpdateTF(auv_config.UTM_LINK, auv_config.BASE_LINK)

        read_mission_plan = ReadTopic(
            name = "A_ReadMissionPlan",
            topic_name = auv_config.PLANDB_TOPIC,
            topic_type = PlanDB,
            # passing None reads the entire message
            blackboard_variables = {bb_enums.MISSION_PLAN_MSG:None}
        )

        read_plan_control = ReadTopic(
            name = "A_ReadPlanControl",
            topic_name= auv_config.PLAN_CONTROL_TOPIC,
            topic_type = PlanControl,
            blackboard_variables = {bb_enums.PLAN_CONTROL_MSG:None}
        )

        set_utm_from_gps = A_SetUTMFromGPS(auv_config.GPS_FIX_TOPIC)

        return Sequence(name="SQ-DataIngestion",
                        children=[
                            read_abort,
                            read_leak,
                            read_depth,
                            read_alt,
                            update_tf,
                            read_mission_plan,
                            read_plan_control,
                            set_utm_from_gps
                        ])



    def const_feedback_tree():
        #TODO separate these 3 into their own actions, this action is just too big
        neptus_feedback = A_PublishToNeptus(auv_config.ESTIMATED_STATE_TOPIC,
                                           auv_config.PLAN_CONTROL_STATE_TOPIC,
                                           auv_config.VEHICLE_STATE_TOPIC)
        # more feedback options will go here

        return Sequence(name="SQ-Feedback",
                        children=[
                                  neptus_feedback
                        ])



    def const_safety_tree():
        no_abort = C_NoAbortReceived()
        altOK = C_AltOK(auv_config.MIN_ALTITUDE)
        depthOK = C_DepthOK(auv_config.MAX_DEPTH)
        leakOK = C_LeakOK()
        # more safety checks will go here

        safety_checks = Sequence(name="SQ-SafetyChecks",
                        children=[
                                  no_abort,
                                  altOK,
                                  depthOK,
                                  leakOK
                        ])

        surface = A_EmergencySurface(auv_config.EMERGENCY_ACTION_NAMESPACE)

        # if anything about safety is 'bad', we abort everything
        fallback_to_abort = Fallback(name='FB_SafetyOK',
                                     children = [
                                         safety_checks,
                                         surface
                                     ])
        return fallback_to_abort



    def const_synch_tree():
        def const_mission_plan_update():
            got_new_plan = C_NewMissionPlanReceived()

            set_new_plan = A_SetMissionPlan(auv_config.UTM_LINK)
            set_next_plan_action = A_SetNextPlanAction()
            set_new_plan_and_action = Sequence(name="SQ-SetNewPlanAndAction",
                                               children=[
                                                   set_new_plan,
                                                   set_next_plan_action
                                               ])

            return Sequence(name="SQ-UpdateMissionPlan",
                            children=[
                                      got_new_plan,
                                      set_new_plan_and_action
                            ])

        def const_mission_synch_gate():
            have_mission = pt.blackboard.CheckBlackboardVariable(name="C_HaveMission",
                                                                 variable_name=bb_enums.MISSION_PLAN_OBJ)
            answer_neptus = A_AnswerNeptusPlanReceived(auv_config.PLANDB_TOPIC)

            return Sequence(name="SQ-MissionSynchronized",
                            children=[
                                      have_mission,
                                      answer_neptus
                            ])

        mission_plan = const_mission_plan_update()
        # more synchronization actions can go here

        synched = const_mission_synch_gate()


        return Fallback(name="FB-SynchroniseMission",
                        children=[
                                  mission_plan,
                                  synched
                        ])


    def const_execute_mission_tree():
        def const_execute_mission_plan():
            plan_complete = C_PlanCompleted()
            # but still wait for operator to tell us to 'go'
            start_received = C_StartPlanReceived()
            execute_plan_action = A_ExecutePlanAction(auv_config.ACTION_NAMESPACE)
            set_next_plan_action = A_SetNextPlanAction()

            follow_plan = Sequence(name="SQ-FollowMissionPlan",
                                   children=[
                                             start_received,
                                             execute_plan_action,
                                             set_next_plan_action
                                   ])

            return Fallback(name="FB-ExecuteMissionPlan",
                            children=[
                                      plan_complete,
                                      follow_plan
                            ])


        mission_plan = const_execute_mission_plan()
        # add more mission actions here

        return Fallback(name="FB-ExecuteMission",
                        children=[
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
                              data_ingestion_tree,
                              feedback_tree,
                              safety_tree,
                              synch_mission_tree,
                              exec_mission_tree,
                              finalize_mission_tree])

    return ptr.trees.BehaviourTree(root)



def main(config):

    utm_zone = rospy.get_param("~utm_zone", common_globals.DEFAULT_UTM_ZONE)
    utm_band = rospy.get_param("~utm_band", common_globals.DEFAULT_UTM_BAND)

    bb = pt.blackboard.Blackboard()
    bb.set(bb_enums.UTM_ZONE, utm_zone)
    bb.set(bb_enums.UTM_BAND, utm_band)



    try:
        rospy.loginfo("Constructing tree")
        tree = const_tree(config)
        rospy.loginfo("Setting up tree")
        tree.setup(timeout=10)
        rospy.loginfo("Ticktocking....")
        while not rospy.is_shutdown():
            # rate is period in ms
            #  tree.tick_tock(1, post_tick_handler=lambda t: pt.display.print_ascii_tree(tree.root, show_status=True))
            tree.tick_tock(common_globals.BT_TICKING_PERIOD)

    except rospy.ROSInitException:
        rospy.loginfo("ROS Interrupt")




if __name__ == '__main__':
    # init the node
    rospy.init_node("bt")
    robot_name = rospy.get_param("~robot_name", "sam")

    if robot_name[:3] == "sam":
        from auv_config import SAMConfig
        config = SAMConfig()
    elif robot_name[:4] == "lolo":
        from auv_config import LOLOConfig
        config = LOLOConfig()
    else:
        rospy.logerr("ROBOT NAME NOT UNDERSTOOD, CONFIG NOT LOADED, EXITING:"+str(robot_name))
        import sys
        sys.exit(1)


    config.BASE_LINK = rospy.get_param("~base_frame", config.BASE_LINK)
    config.UTM_LINK = rospy.get_param("~utm_frame", config.UTM_LINK)

    config.PLANDB_TOPIC = rospy.get_param("~plandb_topic", config.PLANDB_TOPIC)
    config.PLAN_CONTROL_TOPIC = rospy.get_param("~plan_control_topic", config.PLAN_CONTROL_TOPIC)
    config.ESTIMATED_STATE_TOPIC = rospy.get_param("~estimated_state_topic", config.ESTIMATED_STATE_TOPIC)
    config.PLAN_CONTROL_STATE_TOPIC = rospy.get_param("~plan_control_state_topic", config.PLAN_CONTROL_STATE_TOPIC)
    config.VEHICLE_STATE_TOPIC = rospy.get_param("~vehicle_state_topic", config.VEHICLE_STATE_TOPIC)
    config.ABORT_TOPIC = rospy.get_param("~abort_topic", config.ABORT_TOPIC)

    config.ACTION_NAMESPACE = rospy.get_param("~action_namespace", config.ACTION_NAMESPACE)
    config.EMERGENCY_ACTION_NAMESPACE = rospy.get_param("~emergency_action_namespace", config.EMERGENCY_ACTION_NAMESPACE)

    print(config)

    main(config)

