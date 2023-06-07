#! /usr/bin/env python
    # -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import os, time, sys

import rospy


import py_trees as pt
import py_trees_ros as ptr

if not hasattr(rospy, 'loginfo_throttle_identical'): setattr(rospy, 'loginfo_throttle_identical', rospy.loginfo_throttle)
if not hasattr(rospy, 'logwarn_throttle_identical'): setattr(rospy, 'logwarn_throttle_identical', rospy.logwarn_throttle)
if not hasattr(rospy, 'logwarn_once'): setattr(rospy, 'logwarn_once', rospy.logwarn)
if not hasattr(rospy, 'logerr_throttle_identical'):  setattr(rospy, 'logerr_throttle_identical', rospy.logerr_throttle)


# just convenience really
from py_trees.composites import Selector as Fallback

# messages
from std_msgs.msg import Float64, Empty, Bool
from smarc_bt.msg import MissionControl

from auv_config import AUVConfig
from reconfig_server import ReconfigServer

# tree leaves

from bt_conditions import C_DepthOK, \
                          C_NoAbortReceived, \
                          C_AltOK, \
                          C_LeakOK, \
                          C_ExpectPlanState, \
                          C_TimeoutNotReached

from bt_common import Sequence, \
                      CheckBlackboardVariableValue, \
                      ReadTopic, \
                      A_RunOnce, \
                      A_SimplePublisher, \
                      Counter, \
                      Not

from bt_actions import A_GotoWaypoint, \
                       A_SetNextPlanAction, \
                       A_PublishFinalize, \
                       A_ReadWaypoint, \
                       A_AbortPlan



# globally defined values
import bb_enums
import common_globals

# packed up object to keep vehicle-state up to date
# to avoid having a million subscibers inside the tree
from vehicle import Vehicle
from nodered_handler import NoderedHandler

def const_tree(auv_config):
    """
    construct the entire tree.
    the structure of the code reflects the structure of the tree itself.
    sub-trees are constructed in inner functions.
    auv_config is in scope of all these inner functions.

    auv_config is a simple data object with a bunch of UPPERCASE fields in it.
    """
    # slightly hacky way to keep track of 'runnable' actions
    # such actions should add their names to this list in their init
    # just for Neptus vehicle state for now
    bb = pt.blackboard.Blackboard()
    bb.set(bb_enums.MANEUVER_ACTIONS, [])

    # just for clarity when looking at the bb in the field
    bb.set(bb_enums.MISSION_FINALIZED, False)
    bb.set(bb_enums.ROBOT_NAME, auv_config.robot_name)

    def const_data_ingestion_tree():
        read_abort = ptr.subscribers.EventToBlackboard(
            name = "A_ReadAbort",
            topic_name = auv_config.ABORT_TOPIC,
            variable_name = bb_enums.ABORT
        )


         # ReadTopic
         # name,
         # topic_name,
         # topic_type,
         # blackboard_variables,
         # max_period = None,
         # allow_silence = True -> If false, will fail if no message is received ever


        read_reloc_enable = ReadTopic(
            name = "A_ReadLiveWPEnable",
            topic_name = auv_config.LIVE_WP_ENABLE_TOPIC,
            topic_type = Bool,
            blackboard_variables={bb_enums.LIVE_WP_ENABLE : 'data'}
        )

        read_gui_enable = ReadTopic(
            name = "A_ReadGUIEnable",
            topic_name = auv_config.GUI_WP_ENABLE_TOPIC,
            topic_type = Bool,
            blackboard_variables={bb_enums.GUI_WP_ENABLE : 'data'}
        )

        read_algae_follow_enable = ReadTopic(
            name = "A_ReadAlgaeEnable",
            topic_name = auv_config.ALGAE_FOLLOW_ENABLE_TOPIC,
            topic_type = Bool,
            blackboard_variables={bb_enums.ALGAE_FOLLOW_ENABLE: 'data'}
        )


        read_reloc_wp = A_ReadWaypoint(
            ps_topic = auv_config.LIVE_WP,
            bb_key = bb_enums.LIVE_WP,
            utm_to_lat_lon_service_name=auv_config.UTMTOLATLON_SERVICE,
            lat_lon_to_utm_service_name=auv_config.LATLONTOUTM_SERVICE)

        read_algae_follow_wp = A_ReadWaypoint(
            ps_topic = auv_config.ALGAE_FOLLOW_WP,
            bb_key = bb_enums.ALGAE_FOLLOW_WP,
            utm_to_lat_lon_service_name=auv_config.UTMTOLATLON_SERVICE,
            lat_lon_to_utm_service_name=auv_config.LATLONTOUTM_SERVICE)

        read_gui_wp = A_ReadWaypoint(
            ps_topic = auv_config.GUI_WP,
            bb_key = bb_enums.GUI_WP,
            utm_to_lat_lon_service_name=auv_config.UTMTOLATLON_SERVICE,
            lat_lon_to_utm_service_name=auv_config.LATLONTOUTM_SERVICE)


        publish_heartbeat = A_SimplePublisher(topic = auv_config.HEARTBEAT_TOPIC,
                                              message_object = Empty())


        return Sequence(name="SQ_DataIngestion",
                        # dont show all the things inside here
                        blackbox_level=1,
                        children=[
                            publish_heartbeat,
                            read_abort,
                            read_reloc_enable,
                            read_reloc_wp,
                            read_gui_enable,
                            read_gui_wp,
                            read_algae_follow_enable,
                            read_algae_follow_wp
                        ])





    def const_safety_tree():
        safety_checks = Sequence(name="SQ_SafetyChecks",
                        blackbox_level=1,
                        children=[
                            C_NoAbortReceived(),
                            C_AltOK(),
                            C_DepthOK(),
                            C_LeakOK(),
                            C_TimeoutNotReached()
                        ])


        skip_wp = Sequence(name='SQ_CountEmergenciesAndSkip',
                           children = [
                               Counter(n=auv_config.EMERGENCY_TRIALS_BEFORE_GIVING_UP,
                                       name="A_EmergencyCounter",
                                       reset=True),
                               A_SetNextPlanAction()
                           ])

        abort = Sequence(name="SQ_ABORT",
                         children = [
                             A_SimplePublisher(topic=auv_config.ABORT_TOPIC,
                                               message_object = Empty()),
                             A_AbortPlan(),
                             A_GotoWaypoint(auv_config = auv_config,
                                            action_namespace = auv_config.EMERGENCY_ACTION_NAMESPACE,
                                            node_name = 'A_EmergencySurface',
                                            goalless = True)
                         ])



        return Fallback(name='FB_SafetyOK',
                        children = [
                            safety_checks,
                            skip_wp,
                            abort
                        ])




    def const_execute_mission_tree():
        #######################
        # GOTO
        #######################
        goto_action = A_GotoWaypoint(auv_config = auv_config)

        unfinalize = pt.blackboard.SetBlackboardVariable(variable_name = bb_enums.MISSION_FINALIZED,
                                                         variable_value = False,
                                                         name = 'A_MissionFinalized->False')


        # and then execute them in order
        follow_plan = Sequence(name="SQ_FollowMissionPlan",
                               children=[
                                         C_ExpectPlanState(MissionControl.FB_RUNNING),
                                         unfinalize,
                                         goto_action,
                                         A_SetNextPlanAction()
                               ])

        #######################
        # LIVE WP
        #######################
        live_wp_enabled = CheckBlackboardVariableValue(bb_enums.LIVE_WP_ENABLE,
                                                       True,
                                                       "C_LiveWPEnabled")

        goto_live_wp = A_GotoWaypoint(auv_config = auv_config,
                                      node_name="A_GotoLiveWP",
                                      wp_from_bb = bb_enums.LIVE_WP,
                                      live_mode_enabled=True)

        live_wp_tree  = Sequence(name="SQ_FollowLiveWP",
                                 children=[
                                     live_wp_enabled,
                                     goto_live_wp
                                 ])

        #######################
        # GUI WP
        #######################
        gui_wp_enabled = CheckBlackboardVariableValue(bb_enums.GUI_WP_ENABLE,
                                                      True,
                                                      "C_GUIWPEnabled")


        goto_gui_wp = A_GotoWaypoint(auv_config = auv_config,
                                     node_name="A_GotoGUIWP",
                                     wp_from_bb = bb_enums.GUI_WP,
                                     live_mode_enabled=True)

        gui_wp_tree  = Sequence(name="SQ_FollowGUIWP",
                                children=[
                                    gui_wp_enabled,
                                    goto_gui_wp
                                ])


        #######################
        # Algae farm line following
        #######################
        algae_follow_enabled = CheckBlackboardVariableValue(bb_enums.ALGAE_FOLLOW_ENABLE,
                                                            True,
                                                            "C_AlgaeFollowEnabled")


        follow_algae = A_GotoWaypoint(auv_config = auv_config,
                                      node_name="A_FollowAlgae",
                                      wp_from_bb = bb_enums.ALGAE_FOLLOW_WP,
                                      live_mode_enabled = True)

        algae_farm_tree = Sequence(name="SQ_FollowAlgaeFarm",
                                   children=[
                                       algae_follow_enabled,
                                       follow_algae
                                   ])

        #######################
        # until the plan is done
        #######################
        return Fallback(name="FB_ExecuteMissionPlan",
                        children=[
                            gui_wp_tree,
                            live_wp_tree,
                            algae_farm_tree,
                            follow_plan
                        ])



    ###############################################
    # ROOT BEGINS
    ###############################################
    finalize_mission = Sequence(name="SQ_FinalizeMission",
                                children=[
                                          C_ExpectPlanState(MissionControl.FB_COMPLETED),
                                          A_PublishFinalize(topic=auv_config.MISSION_COMPLETE_TOPIC)
                                ])
    planned_mission = const_execute_mission_tree()
    run_tree = Fallback(name="FB_Run",
                        children=[
                            C_ExpectPlanState(MissionControl.FB_STOPPED),
                            finalize_mission,
                            planned_mission
                        ])


    root = Sequence(name='SQ_ROOT',
                    children=[
                              const_data_ingestion_tree(),
                              const_safety_tree(),
                              run_tree
                    ])

    return ptr.trees.BehaviourTree(root,record_rosbag=False)


def main():

    # init the node
    rospy.init_node("bt", log_level=rospy.INFO)

    # create a config object that will handle all the rosparams and such
    # read all the fields from rosparams and literally construct the fields of this object
    config = AUVConfig()
    rospy.loginfo(config)

    # create a dynamic reconfig server that defaults to the
    # configs we already have
    # this will update stuff in the BB
    reconfig = ReconfigServer(config)

    # first construct a vehicle that will hold and sub to most things
    rospy.loginfo("Setting up vehicle")
    vehicle = Vehicle(config)
    tf_listener = None
    while tf_listener is None:
        try:
            rospy.loginfo("Setting up tf_listener for vehicle object before BT")
            tf_listener = vehicle.setup_tf_listener(timeout_secs=common_globals.SETUP_TIMEOUT)
        except Exception as e:
            tf_listener = None
            rospy.logerr("Exception when trying to setup tf_listener for vehicle:\n{}".format(e))

        if tf_listener is None:
            rospy.logerr("TF Listener could not be setup! Is there a UTM frame connected to base link? The BT will not work until this is succesfull. \n retrying in 5s.")
            time.sleep(5)

    # put the vehicle model inside the bb
    bb = pt.blackboard.Blackboard()
    bb.set(bb_enums.VEHICLE_STATE, vehicle)

    # this object will handle all the messages from nodered interface
    # in and out
    nodered_handler = NoderedHandler(config, vehicle, bb)

    # construct the BT with the config and a vehicle model
    rospy.loginfo("Constructing tree")
    tree = const_tree(config)
    rospy.loginfo("Setting up tree")
    setup_ok = False
    # make sure the BT is happy
    while not setup_ok:
        setup_ok = tree.setup(timeout=common_globals.SETUP_TIMEOUT)
        if not setup_ok:
            rospy.logerr("Tree could not be setup! Retrying in 5s!")
            time.sleep(5)


    # print out the config and the BT on screen
    rospy.loginfo(config)
    rospy.loginfo(pt.display.ascii_tree(tree.root))

    # setup the ticking freq and the BlackBoard
    rate = rospy.Rate(common_globals.BT_TICK_RATE)

    rospy.loginfo("Ticktocking....")
    while not rospy.is_shutdown():
        # some info _about the tree_ in the BB.
        # better do this outside the tree
        bb.set(bb_enums.TREE_TIP, tree.tip())

        # update the TF of the vehicle first
        # print(vehicle)
        vehicle.tick(tf_listener)
        mplan = bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mplan is not None:
            mplan.tick()
        nodered_handler.tick()
        # an actual tick, finally.
        tree.tick()
        bb.set(bb_enums.LAST_HEARTBEAT_TIME, time.time())

        # use py-trees-tree-watcher if you can
        #  pt.display.print_ascii_tree(tree.root, show_status=True)
        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        rospy.loginfo("ROS Interrupt")

