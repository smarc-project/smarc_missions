#! /usr/bin/env python
    # -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import os

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
from std_msgs.msg import Float64, Empty
from smarc_msgs.msg import Leak, DVL
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
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
                       A_UpdateTF, \
                       A_EmergencySurface, \
                       A_UpdateNeptusEstimatedState, \
                       A_UpdateNeptusPlanControlState, \
                       A_UpdateNeptusVehicleState, \
                       A_UpdateNeptusPlanDB, \
                       A_UpdateNeptusPlanControl, \
                       A_UpdateMissonForPOI, \
                       A_VizPublishPlan, \
                       A_FollowLeader, \
                       A_SetDVLRunning, \
                       A_ReadBuoys, \
                       A_UpdateMissionLog, \
                       A_SaveMissionLog, \
                       A_ManualMissionLog, \
                       A_PublishFinalize, \
                       A_ReadLolo



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
         # allow_silence = True
        read_alt = ReadTopic(
            name = "A_ReadDVL",
            topic_name = auv_config.DVL_TOPIC,
            topic_type = DVL,
            blackboard_variables = {bb_enums.ALTITUDE:'altitude',
                                    bb_enums.DVL_VELOCITY:'velocity'}
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

        read_latlon = ReadTopic(
            name = "A_ReadLatlon",
            topic_name = auv_config.LATLON_TOPIC,
            topic_type = GeoPoint,
            blackboard_variables = {bb_enums.CURRENT_LATITUDE : 'latitude',
                                    bb_enums.CURRENT_LONGITUDE : 'longitude'}
        )
        read_roll = ReadTopic(
            name = "A_ReadRoll",
            topic_name = auv_config.ROLL_TOPIC,
            topic_type = Float64,
            blackboard_variables = {bb_enums.ROLL : 'data'}
        )
        read_pitch = ReadTopic(
            name = "A_ReadPitch",
            topic_name = auv_config.PITCH_TOPIC,
            topic_type = Float64,
            blackboard_variables = {bb_enums.PITCH : 'data'}
        )
        read_yaw = ReadTopic(
            name = "A_ReadYaw",
            topic_name = auv_config.YAW_TOPIC,
            topic_type = Float64,
            blackboard_variables = {bb_enums.YAW : 'data'}
        )

        read_buoys = A_ReadBuoys(
            topic_name=auv_config.BUOY_TOPIC,
            buoy_link=auv_config.LOCAL_LINK,
            utm_link=auv_config.UTM_LINK,
            latlon_utm_serv=auv_config.LATLONTOUTM_SERVICE
        )

        read_gps = ReadTopic(
            name = "A_ReadGPS",
            topic_name = auv_config.GPS_TOPIC,
            topic_type = NavSatFix,
            blackboard_variables = {bb_enums.RAW_GPS:None},
        )

        read_lolo = A_ReadLolo(
            robot_name = auv_config.robot_name,
            elevator_topic = auv_config.LOLO_ELEVATOR_TOPIC,
            elevon_port_topic = auv_config.LOLO_ELEVON_PORT_TOPIC,
            elevon_strb_topic = auv_config.LOLO_ELEVON_STRB_TOPIC,
            aft_tank_topic = auv_config.LOLO_AFT_TANK_TOPIC,
            front_tank_topic = auv_config.LOLO_FRONT_TANK_TOPIC)


        def const_neptus_tree():
            update_neptus = Sequence(name="SQ-UpdateNeptus",
                                     children=[
                A_UpdateNeptusEstimatedState(auv_config.ESTIMATED_STATE_TOPIC,
                                             auv_config.GPSFIX_TOPIC,
                                             auv_config.GPS_NAV_DATA_TOPIC),
                A_UpdateNeptusPlanControlState(auv_config.PLAN_CONTROL_STATE_TOPIC),
                A_UpdateNeptusVehicleState(auv_config.VEHICLE_STATE_TOPIC),
                A_UpdateNeptusPlanDB(auv_config.PLANDB_TOPIC,
                                     auv_config.UTM_LINK,
                                     auv_config.LOCAL_LINK,
                                     auv_config.LATLONTOUTM_SERVICE,
                                     auv_config.LATLONTOUTM_SERVICE_ALTERNATIVE),
                A_UpdateNeptusPlanControl(auv_config.PLAN_CONTROL_TOPIC),
                A_VizPublishPlan(auv_config.PLAN_VIZ_TOPIC)
                                     ])
            return update_neptus


        update_tf = A_UpdateTF(auv_config.UTM_LINK, auv_config.BASE_LINK)
        neptus_tree = const_neptus_tree()
        publish_heartbeat = A_SimplePublisher(topic = auv_config.HEARTBEAT_TOPIC,
                                              message_object = Empty())


        return Sequence(name="SQ-DataIngestion",
                        # dont show all the things inside here
                        blackbox_level=1,
                        children=[
                            read_abort,
                            read_leak,
                            read_alt,
                            read_detection,
                            read_latlon,
                            read_roll,
                            read_pitch,
                            read_yaw,
                            read_gps,
                            read_buoys,
                            read_lolo,
                            update_tf,
                            neptus_tree,
                            publish_heartbeat
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
                            A_EmergencySurface(auv_config.EMERGENCY_ACTION_NAMESPACE)
                         ])



        return Fallback(name='FB_SafetyOK',
                        children = [
                            safety_checks,
                            skip_wp,
                            abort
                        ])


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
                                                     auv_config.POI_DETECTOR_LINK,
                                                     auv_config.LATLONTOUTM_SERVICE)
                            ])

        return Fallback(name="FB_AutonomousUpdates",
                        children=[
                          C_AutonomyDisabled(),
                          poi_tree
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
        goto_action = A_GotoWaypoint(action_namespace = auv_config.ACTION_NAMESPACE,
                                     goal_tf_frame = auv_config.UTM_LINK)
        wp_is_goto = C_CheckWaypointType(expected_wp_type = imc_enums.MANEUVER_GOTO)
        goto_maneuver = Sequence(name="SQ-GotoWaypoint",
                                 children=[
                                     wp_is_goto,
                                     goto_action
                                 ])


        # SAMPLE
        #XXX USING THE GOTO ACTION HERE TOO UNTIL WE HAVE A SAMPLE ACTION
        sample_action = A_GotoWaypoint(action_namespace = auv_config.ACTION_NAMESPACE,
                                       goal_tf_frame = auv_config.UTM_LINK)
        wp_is_sample = C_CheckWaypointType(expected_wp_type = imc_enums.MANEUVER_SAMPLE)
        sample_maneuver = Sequence(name="SQ-SampleWaypoint",
                                 children=[
                                     wp_is_sample,
                                     sample_action
                                 ])


        #############################################################################################
        # INSPECT
        #TODO add an inspection maneuver  into bridge and neptus etc.
        # wp_is_inspect = C_CheckWaypointType(expected_wp_type = imc_enums.MANEUVER_INSPECT)
        #inspection_action = A_GotoWaypoint(action_namespace = auv_config.INSPECTION_ACTION_NAMESPACE,
        #                                   goal_tf_frame = auv_config.UTM_LINK)
        # inspection_maneuver = Sequence(name="SQ-InspectWP",
                                       # children=[
                                           # wp_is_inspect,
                                           # inspection_action
                                       # ])
        #############################################################################################


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
                                         A_UpdateMissionLog(),
                                         unfinalize,
                                         execute_maneuver,
                                         A_SetNextPlanAction()
                               ])

        # until the plan is done
        return Fallback(name="FB-ExecuteMissionPlan",
                        children=[
                                  C_PlanCompleted(),
                                  follow_plan,
                                  A_SaveMissionLog()
                        ])


    def const_finalize_mission():
        publish_complete = A_PublishFinalize(topic=auv_config.MISSION_COMPLETE_TOPIC)




        unset_plan_is_go = pt.blackboard.SetBlackboardVariable(variable_name = bb_enums.PLAN_IS_GO,
                                                               variable_value = False,
                                                               name = 'A_SetPlanIsGo->False')

        plan_complete_or_stopped = Fallback(name="FB-PlanCompleteOrStopped",
                                            children=[
                                                      C_PlanCompleted(),
                                                      Not(C_StartPlanReceived())
                                            ])


        return Sequence(name="SQ-FinalizeMission",
                        children=[
                                  C_HaveCoarseMission(),
                                  C_PlanIsNotChanged(),
                                  plan_complete_or_stopped,
                                  publish_complete,
                                  unset_plan_is_go,
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
                            #  const_leader_follower()
                        ])

    manual_logging = A_ManualMissionLog(latlontoutm_service_name = auv_config.LATLONTOUTM_SERVICE,
                                        latlontoutm_service_name_alternative = auv_config.LATLONTOUTM_SERVICE_ALTERNATIVE)


    root = Sequence(name='SQ-ROOT',
                    children=[
                              const_data_ingestion_tree(),
                              manual_logging,
                              const_safety_tree(),
                             # const_dvl_tree(),
                              run_tree
                    ])

    return ptr.trees.BehaviourTree(root)


def main():
    package_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.path.pardir)

    config = AUVConfig()
    launch_path = os.path.join(package_path, 'launch', 'smarc_bt.launch')
    try:
        config.generate_launch_file(launch_path)
    except Exception as e:
        print("Did not generate the launch file")
        print(e)

    # init the node
    rospy.init_node("bt", log_level=rospy.INFO)

    # read all the fields from rosparams, lowercased and with ~ prepended
    # this might over-write the defaults in py, as it should
    config.read_rosparams()

    # create a dynamic reconfig server that defaults to the
    # configs we already have
    # this will update stuff in the BB
    reconfig = ReconfigServer(config)

    try:
        rospy.loginfo("Constructing tree")
        tree = const_tree(config)
        rospy.loginfo("Setting up tree")
        setup_ok = tree.setup(timeout=common_globals.SETUP_TIMEOUT)
        viz = pt.display.ascii_tree(tree.root)
        rospy.loginfo(viz)

        # this will put it in the ~/.ros folder if run from launch file
        last_ran_tree_path = 'last_ran_tree.txt'
        with open(last_ran_tree_path, 'w+') as f:
            f.write(viz)
            rospy.loginfo("Wrote the tree to {}".format(last_ran_tree_path))


        if setup_ok:
            rospy.loginfo(config)
            rospy.loginfo("Ticktocking....")
            rate = rospy.Rate(common_globals.BT_TICK_RATE)

            bb = pt.blackboard.Blackboard()

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

                # an actual tick, finally.
                tree.tick()

                # use py-trees-tree-watcher if you can
                #  pt.display.print_ascii_tree(tree.root, show_status=True)
                rate.sleep()

        else:
            rospy.logerr("Tree could not be setup! Exiting!")

    except rospy.ROSInitException:
        rospy.loginfo("ROS Interrupt")



if __name__ == '__main__':
    main()

