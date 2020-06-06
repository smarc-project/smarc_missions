#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import rospy
import py_trees as pt
import py_trees_ros as ptr

# just convenience really
from py_trees.composites import Selector as Fallback

# messages
from std_msgs.msg import Float64
from sam_msgs.msg import Leak
from cola2_msgs.msg import DVL
from geometry_msgs.msg import PointStamped

from auv_config import AUVConfig

# tree leaves
from bt_actions import A_GotoWaypoint, \
                       A_SetNextPlanAction, \
                       A_UpdateTF, \
                       A_EmergencySurface, \
                       A_EmergencySurfaceByForce, \
                       A_SetUTMFromGPS, \
                       A_UpdateLatLon, \
                       A_RefineMission, \
                       A_UpdateNeptusEstimatedState, \
                       A_UpdateNeptusPlanControlState, \
                       A_UpdateNeptusVehicleState, \
                       A_UpdateNeptusPlanDB, \
                       A_UpdateNeptusPlanControl, \
                       A_UpdateMissonForPOI, \
                       A_VizPublishPlan, \
                       A_FollowLeader

from bt_conditions import C_PlanCompleted, \
                          C_NoAbortReceived, \
                          C_DepthOK, \
                          C_AltOK, \
                          C_LeakOK, \
                          C_StartPlanReceived, \
                          C_HaveRefinedMission, \
                          C_HaveCoarseMission, \
                          C_PlanIsNotChanged, \
                          C_NoNewPOIDetected, \
                          C_AutonomyDisabled, \
                          C_LeaderFollowerEnabled, \
                          C_LeaderExists, \
                          C_LeaderIsFarEnough

from bt_common import Sequence, \
                      CheckBlackboardVariableValue, \
                      ReadTopic, \
                      A_RunOnce, \
                      Counter


# globally defined values
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
    # slightly hacky way to keep track of 'runnable' actions
    # such actions should add their names to this list in their init
    # just for Neptus vehicle state for now
    bb = pt.blackboard.Blackboard()
    bb.set(bb_enums.MANEUVER_ACTIONS, [])

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
            topic_type = DVL,
            blackboard_variables = {bb_enums.ALTITUDE:'altitude'}
        )

        read_leak = ReadTopic(
            name = "A_ReadLeak",
            topic_name = auv_config.LEAK_TOPIC,
            topic_type = Leak,
            blackboard_variables = {bb_enums.LEAK:'value'}
        )

        read_detection = ReadTopic(
            name = "A_ReadCameraDetection",
            topic_name = auv_config.CAMERA_DETECTION_TOPIC,
            topic_type = PointStamped,
            blackboard_variables = {bb_enums.POI_POINT_STAMPED:None} # read the entire message into the bb
        )

        def const_neptus_tree():
            update_neptus = Sequence(name="SQ-UpdateNeptus",
                                     children=[
                A_UpdateNeptusEstimatedState(auv_config.ESTIMATED_STATE_TOPIC),
                A_UpdateNeptusPlanControlState(auv_config.PLAN_CONTROL_STATE_TOPIC),
                A_UpdateNeptusVehicleState(auv_config.VEHICLE_STATE_TOPIC),
                A_UpdateNeptusPlanDB(auv_config.PLANDB_TOPIC,
                                     auv_config.UTM_LINK,
                                     auv_config.LOCAL_LINK),
                A_UpdateNeptusPlanControl(auv_config.PLAN_CONTROL_TOPIC),
                A_VizPublishPlan(auv_config.PLAN_VIZ_TOPIC)
                                     ])
            return update_neptus


        update_tf = A_UpdateTF(auv_config.UTM_LINK, auv_config.BASE_LINK)
        update_latlon = A_UpdateLatLon()
        set_utm_from_gps = A_SetUTMFromGPS(auv_config.GPS_FIX_TOPIC)
        neptus_tree = const_neptus_tree()


        return Sequence(name="SQ-DataIngestion",
                        # dont show all the things inside here
                        blackbox_level=1,
                        children=[
                            read_abort,
                            read_leak,
                            read_depth,
                            read_alt,
                            read_detection,
                            set_utm_from_gps,
                            update_tf,
                            update_latlon,
                            neptus_tree
                        ])



    def const_safety_tree():
        no_abort = C_NoAbortReceived()
        altOK = C_AltOK(auv_config.MIN_ALTITUDE)
        depthOK = C_DepthOK(auv_config.MAX_DEPTH)
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

        surface = Fallback(name="FB_Surface",
                           children=[
                               A_EmergencySurface(auv_config.EMERGENCY_ACTION_NAMESPACE),
                               A_EmergencySurfaceByForce(auv_config.EMERGENCY_TOPIC,
                                                         auv_config.VBS_CMD_TOPIC,
                                                         auv_config.RPM_CMD_TOPIC,
                                                         auv_config.LCG_PID_ENABLE_TOPIC,
                                                         auv_config.VBS_PID_ENABLE_TOPIC,
                                                         auv_config.TCG_PID_ENABLE_TOPIC,
                                                         auv_config.YAW_PID_ENABLE_TOPIC,
                                                         auv_config.DEPTH_PID_ENABLE_TOPIC,
                                                         auv_config.VEL_PID_ENABLE_TOPIC)

                           ])

        skip_wp = Sequence(name='SQ-CountEmergenciesAndSkip',
                           children = [
                               Counter(n=auv_config.EMERGENCY_TRIALS_BEFORE_GIVING_UP,
                                       name="A_EmergencyCounter",
                                       reset=True),
                               A_SetNextPlanAction()
                           ])

        # if anything about safety is 'bad', we abort everything
        fallback_to_abort = Fallback(name='FB_SafetyOK',
                                     children = [
                                         safety_checks,
                                         skip_wp,
                                         surface
                                     ])
        return fallback_to_abort


    def const_leader_follower():
        return Sequence(name="SQ_LeaderFollower",
                        children=[
                            C_LeaderFollowerEnabled(config.ENABLE_LEADER_FOLLOWER),
                            C_LeaderExists(config.BASE_LINK,
                                           config.LEADER_LINK),
                            C_LeaderIsFarEnough(config.BASE_LINK,
                                                config.LEADER_LINK,
                                                config.MIN_DISTANCE_TO_LEADER),
                            A_FollowLeader(config.FOLLOW_ACTION_NAMESPACE,
                                           config.LEADER_LINK)
                        ])



    def const_autonomous_updates():
        poi_tree = Fallback(name="FB_Poi",
                            children=[
                                C_NoNewPOIDetected(common_globals.POI_DIST),
                                A_UpdateMissonForPOI(auv_config.UTM_LINK,
                                                     auv_config.LOCAL_LINK,
                                                     auv_config.POI_DETECTOR_LINK)
                            ])

        return Fallback(name="FB_AutonomousUpdates",
                        children=[
                          C_AutonomyDisabled(),
                          poi_tree
                        ])


    def const_synch_tree():
        have_refined_mission = C_HaveRefinedMission()
        have_coarse_mission = C_HaveCoarseMission()
        refine_mission = A_RefineMission(config.PATH_PLANNER_NAME,
                                         config.PATH_TOPIC)
        # we need one here too, to initialize the mission in the first place
        # set dont_visit to True so we dont skip the first wp of the plan
        set_next_plan_action = A_SetNextPlanAction(do_not_visit=True)


        refinement_tree = Sequence(name="SQ_Refinement",
                                   children=[
                                       have_coarse_mission,
                                       refine_mission,
                                       set_next_plan_action
                                   ])

        return Fallback(name='FB_SynchMission',
                        children=[
                            have_refined_mission,
                            refinement_tree
                        ])


    def const_execute_mission_tree():
        plan_complete = C_PlanCompleted()
        # but still wait for operator to tell us to 'go'
        start_received = C_StartPlanReceived()
        gotowp = A_GotoWaypoint(auv_config.ACTION_NAMESPACE)
        # and this will run after every success of the goto action
        set_next_plan_action = A_SetNextPlanAction()
        plan_is_same = C_PlanIsNotChanged()
        idle = pt.behaviours.Running(name="Idle")

        follow_plan = Sequence(name="SQ-FollowMissionPlan",
                               children=[
                                         start_received,
                                         plan_is_same,
                                         gotowp,
                                         set_next_plan_action
                               ])

        return Fallback(name="FB-ExecuteMissionPlan",
                        children=[
                                  plan_complete,
                                  follow_plan,
                                  idle
                        ])



    # The root of the tree is here

    planned_mission = Sequence(name="SQ_PlannedMission",
                               children=[
                                  const_synch_tree(),
                                  const_autonomous_updates(),
                                  const_execute_mission_tree()
                               ])


    run_tree = Fallback(name="FB-Run",
                        children=[
                            planned_mission,
                            const_leader_follower()
                        ])


    root = Sequence(name='SQ-ROOT',
                    children=[
                              const_data_ingestion_tree(),
                              const_safety_tree(),
                              run_tree
                    ])

    return ptr.trees.BehaviourTree(root)



def main(config, catkin_ws_path):

    utm_zone = rospy.get_param("~utm_zone", common_globals.DEFAULT_UTM_ZONE)
    utm_band = rospy.get_param("~utm_band", common_globals.DEFAULT_UTM_BAND)

    bb = pt.blackboard.Blackboard()
    bb.set(bb_enums.UTM_ZONE, utm_zone)
    bb.set(bb_enums.UTM_BAND, utm_band)



    try:
        rospy.loginfo("Constructing tree")
        tree = const_tree(config)
        rospy.loginfo("Setting up tree")
        setup_ok = tree.setup(timeout=common_globals.SETUP_TIMEOUT)
        viz = pt.display.ascii_tree(tree.root)
        rospy.loginfo(viz)
        path = catkin_ws_path+'catkin_ws/src/smarc_missions/smarc_bt/last_ran_tree.txt'
        with open(path, 'w+') as f:
            f.write(viz)
            rospy.loginfo("Wrote the tree to {}".format(path))

        if setup_ok:
            rospy.loginfo("Ticktocking....")
            rate = rospy.Rate(common_globals.BT_TICK_RATE)

            while not rospy.is_shutdown():
                tip = tree.tip()
                if tip is None:
                    bb.set(bb_enums.TREE_TIP_NAME, '')
                    bb.set(bb_enums.TREE_TIP_STATUS, 'Status.X')
                else:
                    bb.set(bb_enums.TREE_TIP_NAME, tip.name)
                    bb.set(bb_enums.TREE_TIP_STATUS, str(tip.status))

                tree.tick()


                rate.sleep()

        else:
            rospy.logerr("Tree could not be setup! Exiting!")

    except rospy.ROSInitException:
        rospy.loginfo("ROS Interrupt")




if __name__ == '__main__':
    # init the node
    rospy.init_node("bt")

    config = AUVConfig()

    # uncomment this to generate bt_sam.launch file from auv_config.py
    # do this after you add a new field into auv_config.py
    # point path to where your catkin_ws is
    import os
    totally_safe_path = os.environ['ROSLISP_PACKAGE_DIRECTORIES']
    catkin_ws_index = totally_safe_path.find('catkin_ws')
    # this is '/home/ozer/smarc/' for me
    catkin_ws_path = totally_safe_path[:catkin_ws_index]
    try:
        config.generate_launch_file(catkin_ws_path)
    except:
        print("Did not generate the launch file")

    # read all the fields from rosparams, lowercased and with ~ prepended
    print('@@@@@@@@@@@@@')
    config.read_rosparams()
    print(config)
    print('@@@@@@@@@@@@@')
    main(config, catkin_ws_path)

