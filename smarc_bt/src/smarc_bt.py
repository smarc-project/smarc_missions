#! /usr/bin/env python
    # -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import os, time

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
from smarc_msgs.msg import Leak, DVL
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, PoseStamped
from geographic_msgs.msg import GeoPoint

from auv_config import AUVConfig
from reconfig_server import ReconfigServer

# tree leaves

from bt_conditions import C_DepthOK, \
                          C_PlanCompleted, \
                          C_NoAbortReceived, \
                          C_AltOK, \
                          C_LeakOK, \
                          C_StartPlanReceived, \
                          C_HaveCoarseMission, \
                          C_PlanIsNotChanged, \
                          C_NoNewPOIDetected, \
                          C_AutonomyDisabled, \
                          C_LeaderFollowerEnabled, \
                          C_LeaderExists, \
                          C_LeaderIsFarEnough, \
                          C_AtDVLDepth, \
                          C_CheckWaypointType

from bt_common import Sequence, \
                      CheckBlackboardVariableValue, \
                      ReadTopic, \
                      A_RunOnce, \
                      A_SimplePublisher, \
                      Counter, \
                      Not

from bt_actions import A_GotoWaypoint, \
                       A_SetNextPlanAction, \
                       A_PublishMissionPlan, \
                       A_FollowLeader, \
                       A_SetDVLRunning, \
                       A_ReadBuoys, \
                       A_UpdateMissionLog, \
                       A_SaveMissionLog, \
                       A_ManualMissionLog, \
                       A_PublishFinalize, \
                       A_ReadLolo, \
                       A_ReadWaypoint



# globally defined values
import bb_enums
import imc_enums
import common_globals

# packed up object to keep vehicle-state up to date
# to avoid having a million subscibers inside the tree
from vehicle import Vehicle
from neptus_handler import NeptusHandler
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


        read_detection = ReadTopic(
            name = "A_ReadCameraDetection",
            topic_name = auv_config.CAMERA_DETECTION_TOPIC,
            topic_type = PointStamped,
            blackboard_variables = {bb_enums.POI_POINT_STAMPED:None} # read the entire message into the bb
        )


        read_buoys = A_ReadBuoys(
            topic_name=auv_config.BUOY_TOPIC,
            buoy_link=auv_config.LOCAL_LINK,
            utm_link=auv_config.UTM_LINK,
            latlon_utm_serv=auv_config.LATLONTOUTM_SERVICE
        )

        read_reloc_enable = ReadTopic(
            name = "A_ReadLiveWPEnable",
            topic_name = auv_config.LIVE_WP_ENABLE_TOPIC,
            topic_type = Bool,
            blackboard_variables={bb_enums.LIVE_WP_ENABLE : 'data'}
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
            utm_to_lat_lon_service_name=auv_config.UTM_TO_LATLON_SERVICE,
            lat_lon_to_utm_service_name=auv_config.LATLONTOUTM_SERVICE)

        read_algae_follow_wp = A_ReadWaypoint(
            ps_topic = auv_config.ALGAE_FOLLOW_WP,
            bb_key = bb_enums.ALGAE_FOLLOW_WP,
            utm_to_lat_lon_service_name=auv_config.UTM_TO_LATLON_SERVICE,
            lat_lon_to_utm_service_name=auv_config.LATLONTOUTM_SERVICE)


        read_lolo = A_ReadLolo(
            robot_name = auv_config.robot_name,
            elevator_topic = auv_config.LOLO_ELEVATOR_TOPIC,
            elevon_port_topic = auv_config.LOLO_ELEVON_PORT_TOPIC,
            elevon_strb_topic = auv_config.LOLO_ELEVON_STRB_TOPIC,
            aft_tank_topic = auv_config.LOLO_AFT_TANK_TOPIC,
            front_tank_topic = auv_config.LOLO_FRONT_TANK_TOPIC)



        publish_heartbeat = A_SimplePublisher(topic = auv_config.HEARTBEAT_TOPIC,
                                              message_object = Empty())


        return Sequence(name="SQ-DataIngestion",
                        # dont show all the things inside here
                        blackbox_level=1,
                        children=[
                            read_abort,
                            read_detection,
                            read_buoys,
                            read_lolo,
                            publish_heartbeat,
                            read_reloc_enable,
                            read_reloc_wp,
                            read_algae_follow_enable,
                            read_algae_follow_wp
                        ])


    def const_dvl_tree():
        switch_on = Sequence(name="SQ_SwitchOnDVL",
                             children=[
                                 C_AtDVLDepth(auv_config.DVL_RUNNING_DEPTH),
                                 A_SetDVLRunning(auv_config.START_STOP_DVL_NAMESPACE,
                                                 True,
                                                 auv_config.DVL_COOLDOWN)
                             ])

        switch_off = Fallback(name="FB_SwitchOffDVL",
                              children=[
                                  switch_on,
                                  A_SetDVLRunning(auv_config.START_STOP_DVL_NAMESPACE,
                                                  False,
                                                  auv_config.DVL_COOLDOWN)
                              ])

        return switch_off



    def const_safety_tree():
        no_abort = C_NoAbortReceived()
        altOK = C_AltOK()
        depthOK = C_DepthOK()
        leakOK = C_LeakOK()
        # more safety checks will go here

        safety_checks = Sequence(name="SQ-SafetyChecks",
                        blackbox_level=1,
                        children=[
                                  no_abort,
                                  altOK,
                                  depthOK,
                                  leakOK
                        ])


        skip_wp = Sequence(name='SQ-CountEmergenciesAndSkip',
                           children = [
                               Counter(n=auv_config.EMERGENCY_TRIALS_BEFORE_GIVING_UP,
                                       name="A_EmergencyCounter",
                                       reset=True),
                               A_SetNextPlanAction()
                           ])

        abort = Sequence(name="SQ-ABORT",
                         children = [
                            A_SimplePublisher(topic=auv_config.EMERGENCY_TOPIC,
                                              message_object = Empty()),
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



    def const_synch_tree():
        have_coarse_mission = C_HaveCoarseMission()
        # we need one here too, to initialize the mission in the first place
        # set dont_visit to True so we dont skip the first wp of the plan
        # and simply ready the bb to have the waypoint in it
        set_next_plan_action = A_SetNextPlanAction(do_not_visit=True)

        return Sequence(name="SQ_GotMission",
                        children=[
                            have_coarse_mission,
                            set_next_plan_action
                        ])




    def const_execute_mission_tree():
        # GOTO
        goto_action = A_GotoWaypoint(auv_config = auv_config)
        mission_wp_is_goto = C_CheckWaypointType(expected_wp_type = imc_enums.MANEUVER_GOTO)
        goto_maneuver = Sequence(name="SQ-GotoWaypoint",
                                 children=[
                                     mission_wp_is_goto,
                                     goto_action
                                 ])


        # SAMPLE
        #XXX USING THE GOTO ACTION HERE TOO UNTIL WE HAVE A SAMPLE ACTION
        sample_action = A_GotoWaypoint(auv_config = auv_config,
                                       node_name = "A_SampleWaypoint")
        mission_wp_is_sample = C_CheckWaypointType(expected_wp_type = imc_enums.MANEUVER_SAMPLE)
        sample_maneuver = Sequence(name="SQ-SampleWaypoint",
                                 children=[
                                     mission_wp_is_sample,
                                     sample_action
                                 ])



        # put the known plannable maneuvers in here as each others backups
        execute_maneuver = Fallback(name="FB-ExecuteManeuver",
                                    children=[
                                        goto_maneuver,
                                        sample_maneuver
                                    ])

        unfinalize = pt.blackboard.SetBlackboardVariable(variable_name = bb_enums.MISSION_FINALIZED,
                                                         variable_value = False,
                                                         name = 'A_MissionFinalized->False')


        # and then execute them in order
        follow_plan = Sequence(name="SQ-FollowMissionPlan",
                               children=[
                                         C_HaveCoarseMission(),
                                         C_StartPlanReceived(),
                                         unfinalize,
                                         execute_maneuver,
                                         A_SetNextPlanAction()
                               ])

        live_wp_enabled = CheckBlackboardVariableValue(bb_enums.LIVE_WP_ENABLE,
                                                       True,
                                                       "C_LiveWPEnabled")

        live_wp_is_goto = C_CheckWaypointType(expected_wp_type = imc_enums.MANEUVER_GOTO,
                                              bb_key = bb_enums.LIVE_WP)

        goto_live_wp = A_GotoWaypoint(auv_config = auv_config,
                                      node_name="A_GotoLiveWP",
                                      wp_from_bb = bb_enums.LIVE_WP,
                                      live_mode_enabled=True)

        live_wp_tree  = Sequence(name="SQ-FollowLiveWP",
                                 children=[
                                     live_wp_enabled,
                                     live_wp_is_goto,
                                     goto_live_wp
                                 ])

        # Algae farm line following
        algae_follow_enabled = CheckBlackboardVariableValue(bb_enums.ALGAE_FOLLOW_ENABLE,
                                                            True,
                                                            "C_AlgaeFollowEnabled")

        algae_wp_is_goto = C_CheckWaypointType(expected_wp_type = imc_enums.MANEUVER_GOTO,
                                               bb_key = bb_enums.ALGAE_FOLLOW_WP)

        follow_algae = A_GotoWaypoint(auv_config = auv_config,
                                      node_name="A_FollowAlgae",
                                      wp_from_bb = bb_enums.ALGAE_FOLLOW_WP,
                                      live_mode_enabled = True)

        algae_farm_tree = Sequence(name="SQ-FollowAlgaeFarm",
                                   children=[
                                       algae_follow_enabled,
                                       algae_wp_is_goto,
                                       follow_algae
                                   ])


        # until the plan is done
        return Fallback(name="FB-ExecuteMissionPlan",
                        children=[
                                  live_wp_tree,
                                  algae_farm_tree,
                                  C_PlanCompleted(),
                                  follow_plan
                        ])


    def const_finalize_mission():
        publish_complete = A_PublishFinalize(topic=auv_config.MISSION_COMPLETE_TOPIC)

        plan_complete_or_stopped = Fallback(name="FB-PlanCompleteOrStopped",
                                            children=[
                                                      C_PlanCompleted(),
                                                      Not(C_StartPlanReceived())
                                            ])



        return Sequence(name="SQ-FinalizeMission",
                        children=[
                                  C_HaveCoarseMission(),
                                  C_PlanIsNotChanged(),
                                  C_StartPlanReceived(),
                                  A_UpdateMissionLog(),
                                  plan_complete_or_stopped,
                                  publish_complete,
                                  A_SaveMissionLog()
                        ])

    # The root of the tree is here


    planned_mission = const_execute_mission_tree()


    # use this to kind of set the tree to 'idle' mode that wont attempt
    # to control anything and just chills as an observer
    # finalized = CheckBlackboardVariableValue(bb_enums.MISSION_FINALIZED,
                                             # True,
                                             # "C_MissionFinalized")

    run_tree = Fallback(name="FB-Run",
                        children=[
                            # finalized,
                            const_finalize_mission(),
                            planned_mission
                        ])

    manual_logging = A_ManualMissionLog(config = auv_config)


    root = Sequence(name='SQ-ROOT',
                    children=[
                              const_data_ingestion_tree(),
                              manual_logging,
                              const_safety_tree(),
                             # const_dvl_tree(),
                              run_tree
                    ])

    return ptr.trees.BehaviourTree(root,record_rosbag=False)


def main():
    # create a config object that will handle all the rosparams and such
    # and then auto-generate the launch file from it
    config = AUVConfig()
    package_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.path.pardir)
    launch_path = os.path.join(package_path, 'launch', 'smarc_bt.launch')
    try:
        config.generate_launch_file(launch_path)
    except Exception as e:
        print("Did not generate the launch file, will continue: \n{}".format(e))

    # init the node
    rospy.init_node("bt", log_level=rospy.INFO)

    # read all the fields from rosparams, lowercased and with ~ prepended
    # this might over-write the defaults in py, as it should
    config.read_rosparams()

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
            tf_listener = vehicle.setup_tf_listener(timeout_secs=common_globals.SETUP_TIMEOUT)
        except Exception as e:
            tf_listener = None
            rospy.logerr("Exception when trying to setup tf_listener for vehicle:\n{}".format(e))

        if tf_listener is None:
            rospy.logerr("TF Listener could not be setup! Is there a UTM frame connected to base link? \n retrying in 5s.")
            time.sleep(5)

    # put the vehicle model inside the bb
    bb = pt.blackboard.Blackboard()
    bb.set(bb_enums.VEHICLE_STATE, vehicle)

    # construct the neptus handler that handles talking to neptus
    # since the BT doesnt really care about the stuff from neptus beyond
    # signals, it doesnt need these as actions and such
    neptus_handler = NeptusHandler(config, vehicle, bb)
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
            sleep(5)


    # write the structure of the tree to file, useful for post-mortem inspections
    # if needed
    # this will put it in the ~/.ros folder if run from launch file
    last_ran_tree_path = 'last_ran_tree.txt'
    bt_viz = pt.display.ascii_tree(tree.root)
    with open(last_ran_tree_path, 'w+') as f:
        f.write(bt_viz)
        rospy.loginfo("Wrote the tree to {}".format(last_ran_tree_path))


    # print out the config and the BT on screen
    rospy.loginfo(config)
    rospy.loginfo(bt_viz)

    # setup the ticking freq and the BlackBoard
    rate = rospy.Rate(common_globals.BT_TICK_RATE)

    rospy.loginfo("Ticktocking....")
    while not rospy.is_shutdown():
        # some info _about the tree_ in the BB.
        # better do this outside the tree
        tip = tree.tip()
        if tip is None:
            bb.set(bb_enums.TREE_TIP_NAME, '')
            bb.set(bb_enums.TREE_TIP_STATUS, 'Status.X')
        else:
            bb.set(bb_enums.TREE_TIP_NAME, tip.name)
            bb.set(bb_enums.TREE_TIP_STATUS, str(tip.status))

        # update the TF of the vehicle first
        # print(vehicle)
        vehicle.tick(tf_listener)
        # print(neptus_handler)
        neptus_handler.tick()
        nodered_handler.tick()
        # an actual tick, finally.
        tree.tick()

        # use py-trees-tree-watcher if you can
        #  pt.display.print_ascii_tree(tree.root, show_status=True)
        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        rospy.loginfo("ROS Interrupt")

