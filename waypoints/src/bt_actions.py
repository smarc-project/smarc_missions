#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)
# Mostly a re-write of Christopher's behaviours with more
# descriptive names and atomicaztion of everything
# and some new stuff as time goes on

#TODO:
# . A_SetManualWaypoint
# . A_GotoManualWaypoint

# imports courtesy of Chris. TODO cleanup
import py_trees as pt, py_trees_ros as ptr, std_msgs.msg, copy, json, numpy as np
from geodesy.utm import fromLatLong, UTMPoint
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from imc_ros_bridge.msg import PlanControlState
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Bool, Empty
from sam_msgs.msg import PercentStamped


from imc_ros_bridge.msg import EstimatedState, VehicleState, PlanDB, PlanDBInformation, PlanDBState


import actionlib_msgs.msg as actionlib_msgs

import rospy
import tf

import sam_globals
from sam_globals import *

from bt_common import *

class MissionPlan:
    def __init__(self,
                 actions,
                 frame,
                 plan_id=None,
                 original_planddb_message=None):
        """
        A container object to keep things related to the mission plan.
        actions is a list of (x,y,z,type:string) tuples
        """
        self.waypoints = []

        # extract the waypoints only
        for a in actions:
            self.waypoints.append(a[:3])

        self.frame = frame

        if plan_id is None:
            plan_id = "Follow "+str(len(self.waypoints))+" waypoints"
            if original_planddb_message is not None:
                plan_id = original_planddb_message.plan_id
        self.plan_id = plan_id

        self.remaining_wps = self.waypoints
        self.visited_wps = []
        self.current_wp = None
        self.current_wp_index = -1

        # keep it around just in case
        self.original_planddb_message = original_planddb_message

        # used to report when the mission was received
        self.creation_time = time.time()

        self.completed = False

    def __str__(self):
        return 'wps:'+str(self.waypoints)+'\nremaining:'+str(self.remaining_wps)

    def pop_wp(self):
        """
        pop a wp from the remaining wps and return it
        """
        if len(self.remaining_wps) > 0:
            self.current_wp = self.remaining_wps[0]
            self.remaining_wps = self.remaining_wps[1:]
            self.current_wp_index += 1
        else:
            self.completed = True
            self.current_wp = None

        return self.current_wp, self.frame

    def visit(self):
        self.visited_wps.append(self.current_wp)
        self.current_wp = None


class A_SetUTMFromGPS(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Read GPS fix and set our utm band and zone from it.
        Warn when there is a change in it.

        Returns RUNNING until a GPS fix is read.
        Returns SUCCESS afterwards.
        """

        self.bb = pt.blackboard.Blackboard()
        self.gps_sub = rospy.Subscriber(sam_globals.GPS_FIX_TOPIC, NavSatFix, callback=self.gps_fix_cb)
        super(A_SetUTMFromGPS, self).__init__("A_SetUTMFromGPS")

        self.gps_zone = None
        self.gps_band = None


    def gps_fix_cb(self, data):
        self.gps_zone, self.gps_band = fromLatLong(data.latitude, data.longitude).gridZone()


    def update(self):
        if self.gps_zone is None or self.gps_band is None:
            return pt.Status.RUNNING

        # first read the UTMs given by ros params
        prev_band = self.bb.get(UTM_BAND_BB)
        prev_zone = self.bb.get(UTM_ZONE_BB)

        if prev_zone != self.gps_zone or prev_band != self.gps_band:
            rospy.logwarn_once("PREVIOUS UTM AND GPS_FIX UTM ARE DIFFERENT!")
            if sam_globals.TRUST_GPS:
                rospy.logwarn_once("USING GPS UTM!")
                self.bb.set(UTM_ZONE_BB, self.gps_zone)
                self.bb.set(UTM_BAND_BB, self.gps_band)
            else:
                rospy.logwarn_once("USING PREVIOUS UTM!")
                self.bb.set(UTM_ZONE_BB, prev_zone)
                self.bb.set(UTM_BAND_BB, prev_band)

        return pt.Status.SUCCESS



class A_EmergencySurface(ptr.actions.ActionClient):
    def __init__(self):
        """
        Calls Harsha's wp_depth_action_planner action
        """
        self.bb = pt.blackboard.Blackboard()
        self.action_goal_handle = None

        ptr.actions.ActionClient.__init__(
            self,
            name="A_EmergencySurface",
            action_spec=MoveBaseAction,
            action_goal=None,
            action_namespace= sam_globals.EMERGENCY_ACTION_NAMESPACE,
            override_feedback_message_on_running="EMERGENCY SURFACING"
        )

    def initialise(self):
        rospy.logwarn("EMERGENCY SURFACING")
        # construct the message
        self.action_goal = MoveBaseGoal()
        self.sent_goal = False

    def update(self):
        # if your action client is not valid
        if not self.action_client:
            self.feedback_message = "ActionClient for emergency action is invalid!"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if the action_goal is invalid
        if not self.action_goal:
            self.feedback_message = "No action_goal!"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if goal hasn't been sent yet
        if not self.sent_goal:
            self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
            self.sent_goal = True
            rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
            self.feedback_message = "Emergency goal sent"
            self.bb.set(CURRENTLY_RUNNING_ACTION, 'A_EmergencySurface')
            return pt.Status.RUNNING


        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            self.feedback_message = "Aborted emergency"
            rospy.loginfo(self.feedback_message)
            self.bb.set(CURRENTLY_RUNNING_ACTION, None)
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result:
            self.feedback_message = "Completed emergency"
            rospy.loginfo(self.feedback_message)
            self.bb.set(CURRENTLY_RUNNING_ACTION, None)
            return pt.Status.SUCCESS


        # if we're still trying to accomplish the goal
        self.bb.set(CURRENTLY_RUNNING_ACTION, 'A_EmergencySurface')
        return pt.Status.RUNNING

    def feedback_cb(self, msg):
        self.bb.set(LAST_PLAN_ACTION_FEEDBACK, msg)


class A_AnswerNeptusPlanReceived(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Gives neptus feedback that the plan has been received by the vehicle.
        There is a whole conversation that happens between vehicle and neptus
        when a plan is sent over. This node handles that entire conversation.
        Stops the pop-ups that keep telling us that the plan is not synchronized.
        """
        self.bb = pt.blackboard.Blackboard()
        super(A_AnswerNeptusPlanReceived, self).__init__('A_AnswerNeptusPlanReceived')

        # the message body is largely the same, so we can re-use most of it
        self.plandb_msg = PlanDB()
        self.plandb_msg.type = IMC_PLANDB_TYPE_SUCCESS
        self.plandb_msg.op = IMC_PLANDB_OP_SET

        self.plandb_pub = rospy.Publisher(sam_globals.PLAN_TOPIC, PlanDB, queue_size=1)
        self.plandb_sub = rospy.Subscriber(sam_globals.PLAN_TOPIC, PlanDB, callback=self.plandb_cb, queue_size=1)


        # throttle this a little
        self._last_answered_plan_id = None
        self._answer_every_nth = 500
        self._answer_attempt_count = 500


    def make_plandb_info(self):
        current_mission_plan = self.bb.get(MISSION_PLAN_OBJ_BB)
        plan_info = PlanDBInformation()
        plan_info.plan_id = current_mission_plan.original_planddb_message.plan_id
        plan_info.md5 = current_mission_plan.original_planddb_message.plan_spec_md5
        rospy.loginfo_throttle_identical(5, "Sent md5:"+plan_info.md5)
        plan_info.change_time = current_mission_plan.creation_time/1000.0
        return plan_info



    def plandb_cb(self, plandb_msg):
        """
        as an answer to OUR answer of 'type=succes, op=set', neptus sends a 'type=request, op=get_info'.
        """
        typee = plandb_msg.type
        op = plandb_msg.op

        if typee == IMC_PLANDB_TYPE_REQUEST and op == IMC_PLANDB_OP_GET_INFO:
            # we need to respond to this with some info... but what?
            rospy.loginfo_throttle_identical(5, "Got REQUEST GET_INFO planDB msg from Neptus")

            current_mission_plan = self.bb.get(MISSION_PLAN_OBJ_BB)
            if current_mission_plan is None:
                return

            response = PlanDB()
            response.plan_id = current_mission_plan.original_planddb_message.plan_id
            response.type = IMC_PLANDB_TYPE_SUCCESS
            response.op = IMC_PLANDB_OP_GET_INFO
            response.plandb_information = self.make_plandb_info()
            self.plandb_pub.publish(response)
            rospy.loginfo_throttle_identical(5, "Answered GET_INFO with:\n"+str(response))

        elif typee == IMC_PLANDB_TYPE_REQUEST and op == IMC_PLANDB_OP_GET_STATE:
            rospy.loginfo_throttle_identical(5, "Got REQUEST GET_STATE planDB msg from Neptus")
            current_mission_plan = self.bb.get(MISSION_PLAN_OBJ_BB)
            if current_mission_plan is None:
                return


            # https://github.com/LSTS/imcjava/blob/d95fddeab4c439e603cf5e30a32979ad7ace5fbc/src/java/pt/lsts/imc/adapter/PlanDbManager.java#L160
            # See above for an example
            # TODO it seems like we need to keep a planDB ourselves on this side, collect all the plans we
            # received and answer this get_state with data from them all.
            # lets try telling neptus that we just got one plan, maybe that'll be okay?
            # seems alright, but after this message is sent, the plan goes red :/
            response = PlanDB()
            response.plan_id = current_mission_plan.original_planddb_message.plan_id
            response.type = IMC_PLANDB_TYPE_SUCCESS
            response.op = IMC_PLANDB_OP_GET_STATE

            response.plandb_state = PlanDBState()
            response.plandb_state.plan_count = 1
            response.plandb_state.plans_info.append(self.make_plandb_info())

            self.plandb_pub.publish(response)
            rospy.loginfo_throttle_identical(5, "Answered GET_STATE with:\n"+str(response))



        elif typee == IMC_PLANDB_TYPE_SUCCESS:
            rospy.loginfo_throttle_identical(1, "Received SUCCESS for op:"+str(op))

        else:
            rospy.loginfo_throttle_identical(1, "Received some new planDB message:\n"+str(plandb_msg))




    def answer(self, plan_id):
        # just update the plan_id, thats all neptus looks at after type and op
        rospy.loginfo_throttle_identical(5, "Answered Neptus for PlanDB")
        self.plandb_msg.plan_id = plan_id
        self.plandb_pub.publish(self.plandb_msg)


    def update(self):
        current_mission_plan = self.bb.get(MISSION_PLAN_OBJ_BB)
        if current_mission_plan is None:
            return pt.Status.FAILURE

        plan_id = current_mission_plan.original_planddb_message.plan_id
        if plan_id == self._last_answered_plan_id:
            # its the same plan_id we answered
            if self._answer_every_nth - self._answer_attempt_count <= 0:
                # but we havent answered in a while

                self.answer(plan_id)
                self._answer_attempt_count = 0
                return pt.Status.SUCCESS
            else:
                # we answered too recently
                self._answer_attempt_count += 1
                return pt.Status.SUCCESS
        else:
            # we changed plans apparently, answer asap
            self.answer(plan_id)
            self._last_answered_plan_id = plan_id
            self._answer_attempt_count = 0
            return pt.Status.SUCCESS






class A_SetMissionPlan(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Reads the mission plan string from the black board
        and creates a new Mission object.

        returns SUCCESS if there was a plan in MISSION_PLAN_STR and we successfully set it
        returns FAILURE otherwise
        """
        self.bb = pt.blackboard.Blackboard()
        super(A_SetMissionPlan, self).__init__('A_SetMissionPlan')

    def update(self):
        plandb = self.bb.get(MISSION_PLAN_MSG_BB)

        # there was no plan to be set
        if plandb is None:
            self.logger.info("No plan")
            return pt.Status.FAILURE

        # ignore other plan actions for now
        if plandb.op == IMC_PLANDB_OP_SET:
            # there is a plan we can at least look at
            wps_types, frame = self.read_plandb(plandb)

            mission_plan = MissionPlan(actions=wps_types,
                                       frame = frame,
                                       original_planddb_message=plandb)

            self.bb.set(MISSION_PLAN_OBJ_BB, mission_plan)
            self.logger.info("Set the mission plan to:"+str(mission_plan.waypoints))

            return pt.Status.SUCCESS
        else:
            self.logger.info("The accepted mission plandb message was not a SET!")
            return pt.Status.FAILURE

    # TODO move into the  MIssion Plan object instead
    @staticmethod
    def read_plandb(plandb):
        """
        planddb message is a bunch of nested objects,
        we want a list of waypoints and types for now,
        the utm zone and band and the frame in which the mission is defined
        """
        wps_types = []
        frame = '/world_utm'

        request_id = plandb.request_id
        plan_id = plandb.plan_id
        plan_spec = plandb.plan_spec

        for plan_man in plan_spec.maneuvers:
            man_id = plan_man.maneuver_id
            man_name = plan_man.maneuver.maneuver_name
            man_imc_id = plan_man.maneuver.maneuver_imc_id
            maneuver = plan_man.maneuver
            # probably every maneuver has lat lon z in them, but just in case...
            if man_imc_id == IMC_MANEUVER_GOTO:
                lat = maneuver.lat
                lon = maneuver.lon
                depth = maneuver.z
                # w/e f is...
                f = fromLatLong(np.degrees(lat), np.degrees(lon)).toPoint()
                f = (f.x, f.y, depth, IMC_MANEUVER_GOTO)
                wps_types.append(f)
            else:
                print("UNIMPLEMENTED MANEUVER:", man_imc_id, man_name)

        return wps_types, frame


class A_SetNextPlanAction(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Sets the current plan action to the next one
        RUNNING if it can set it to something that is not None
        FAILURE otherwise
        """
        self.bb = pt.blackboard.Blackboard()
        super(A_SetNextPlanAction, self).__init__('A_SetNextPlanAction')

    def update(self):
        mission_plan = self.bb.get(MISSION_PLAN_OBJ_BB)
        next_action = mission_plan.pop_wp()
        if next_action is None:
            self.feedback_message = "Next action was None"
            return pt.Status.FAILURE

        self.logger.info("Set CURRENT_PLAN_ACTION to:"+str(next_action))
        self.bb.set(CURRENT_PLAN_ACTION, next_action)

        self.bb.set(CURRENTLY_RUNNING_ACTION, 'A_SetNextPlanAction')
        return pt.Status.RUNNING



class A_ExecutePlanAction(ptr.actions.ActionClient):
    def __init__(self):
        """
        Executes the currently set plan action in the blackboard

        Copied from Chris's code mostly
        """
        #TODO support more than waypoints
        # in progress with the introduvtion of a proper planDB message

        self.bb = pt.blackboard.Blackboard()

        self.action_goal_handle = None

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name="A_ExecutePlanAction",
            action_spec=MoveBaseAction,
            action_goal=None,
            action_namespace = sam_globals.ACTION_NAMESPACE,
            override_feedback_message_on_running="Moving to waypoint"
        )


    def initialise(self):
        wp, frame = self.bb.get(CURRENT_PLAN_ACTION)
        # if this is the first ever action, we need to get it ourselves
        if wp is None:
            rospy.logwarn("No action found to execute! Was A_SetNextPlanAction called before this?")
            return

        # construct the message
        self.action_goal = MoveBaseGoal()
        self.action_goal.target_pose.pose.position.x = wp[0]
        self.action_goal.target_pose.pose.position.y = wp[1]
        self.action_goal.target_pose.pose.position.z = wp[2]
        self.action_goal.target_pose.header.frame_id = frame
        rospy.loginfo("Action goal initialized")

        # ensure that we still need to send the goal
        self.sent_goal = False

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        # if your action client is not valid
        if not self.action_client:
            self.feedback_message = "ActionClient is invalid!"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if the action_goal is invalid
        if not self.action_goal:
            self.feedback_message = "No action_goal!"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if goal hasn't been sent yet
        if not self.sent_goal:
            self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
            self.sent_goal = True
            rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
            self.feedback_message = "Goal sent"
            self.bb.set(CURRENTLY_RUNNING_ACTION, 'A_ExecutePlanAction')
            return pt.Status.RUNNING


        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            self.feedback_message = "Aborted goal"
            rospy.loginfo(self.feedback_message)
            self.bb.set(CURRENTLY_RUNNING_ACTION, None)
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result:
            self.feedback_message = "Completed goal"
            rospy.loginfo(self.feedback_message)
            self.bb.set(CURRENTLY_RUNNING_ACTION, None)
            return pt.Status.SUCCESS


        # if we're still trying to accomplish the goal
        self.bb.set(CURRENTLY_RUNNING_ACTION, 'A_ExecutePlanAction')
        return pt.Status.RUNNING

    def feedback_cb(self, msg):
        self.bb.set(LAST_PLAN_ACTION_FEEDBACK, msg)


class A_UpdateTF(pt.behaviour.Behaviour):
    def __init__(self):
        """
        reads the current translation and orientation from the TF tree
        and puts that into the BB
        """
        self.bb = pt.blackboard.Blackboard()

        self.listener = tf.TransformListener()

        super(A_UpdateTF, self).__init__("A_UpdateTF")

    def setup(self, timeout):
        try:
            self.listener.waitForTransform("world_utm", sam_globals.BASE_LINK, rospy.Time(), rospy.Duration(4.0))
            return True
        except:
            self.logger.error("Could not find TF!!")
            return False


    def update(self):
        try:
            now = rospy.Time(0)
            (world_trans, world_rot) = self.listener.lookupTransform("world_utm",
                                                                     sam_globals.BASE_LINK,
                                                                     now)
        except (tf.LookupException, tf.ConnectivityException):
            self.logger.warning("Could not get transform between world_utm and "+ sam_globals.BASE_LINK)
            return pt.Status.FAILURE
        except:
            self.logger.warning("Could not do tf lookup for some other reason")
            return pt.Status.FAILURE

        self.bb.set(WORLD_TRANS_BB, world_trans)
        self.bb.set(WORLD_ROT_BB, world_rot)

        return pt.Status.SUCCESS


class A_PublishToNeptus(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Publish some feedback to Neptus.
        Always returns SUCCESS
        """
        self.bb = pt.blackboard.Blackboard()
        self.estimated_state_pub = rospy.Publisher(sam_globals.ESTIMATED_STATE_TOPIC, EstimatedState, queue_size=1)
        self.plan_control_state_pub = rospy.Publisher(sam_globals.PLAN_CONTROL_STATE_TOPIC, PlanControlState, queue_size=1)
        self.vehicle_state_pub = rospy.Publisher(sam_globals.VEHICLE_STATE_TOPIC, VehicleState, queue_size=1)

        super(A_PublishToNeptus, self).__init__("A_PublishToNeptus")

    def update_vehicle_state(self):
        """
        this is the message that makes SAM:DISCONNECTED better.
        """

        vs = VehicleState()

        currently_running = self.bb.get(CURRENTLY_RUNNING_ACTION)
        if currently_running == 'A_ExecutePlanAction':
            vs.op_mode = IMC_OP_MODE_MANEUVER
        elif currently_running == 'A_EmergencySurface':
            vs.op_mode = IMC_OP_MODE_ERROR
        else:
            vs.op_mode = IMC_OP_MODE_SERVICE

        self.vehicle_state_pub.publish(vs)



    def update_estimated_state(self):
        world_rot = self.bb.get(WORLD_ROT_BB)
        world_trans = self.bb.get(WORLD_TRANS_BB)
        depth = self.bb.get(DEPTH_BB)

        # get the utm zone of our current lat,lon
        utmz = self.bb.get(UTM_ZONE_BB)
        band = self.bb.get(UTM_BAND_BB)

        if utmz is None or band is None:
            reason = "Utmz or band was None!!"
            rospy.loginfo(reason)
            self.feedback_message = reason
            return pt.Status.FAILURE

        if world_rot is None or world_trans is None:
            reason = "world_rot or world_trans was None!!"
            rospy.loginfo(reason)
            self.feedback_message = reason
            return pt.Status.FAILURE

        if depth is None:
            reason = "depth was None!!"
            self.feedback_message = reason
            depth = 0



        # XXX here be dragons
        # get positional feedback of the p2p goal
        #  orientation = Quaternion(*world_rot)
        easting, northing = world_trans[0], world_trans[1]
        # make utm point
        pnt = UTMPoint(easting=easting, northing=northing, altitude=0, zone=utmz, band=band)
        # get lat-lon
        pnt = pnt.toMsg()

        # construct message for neptus
        #  mmsg = Pose()
        #  mmsg.position.x = np.radians(pnt.longitude)
        #  mmsg.position.y = np.radians(pnt.latitude)
        #  mmsg.position.z = depth
        #  mmsg.orientation = orientation
        e_state = EstimatedState()
        e_state.lat = np.radians(pnt.latitude)
        e_state.lon= np.radians(pnt.longitude)
        e_state.depth = depth
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(world_rot)
        e_state.psi = np.pi/2. - yaw

        # send the message to neptus
        self.estimated_state_pub.publish(e_state)

    def update_plan_control_state(self):
        # construct current progress message for neptus
        msg = PlanControlState()

        mission_plan = self.bb.get(MISSION_PLAN_OBJ_BB)
        if mission_plan is None or mission_plan.completed:
            msg.plan_id = 'No plan'
            msg.man_id = 'Idle'
            msg.plan_progress = 100.0
        else:
            current_wp_index = mission_plan.current_wp_index
            current_wp = mission_plan.current_wp
            total = len(mission_plan.waypoints)
            plan_progress = (current_wp_index * 100.0) / total # percent float
            msg.plan_id = str(mission_plan.plan_id)
            msg.man_id = 'Goto'+str(current_wp_index+1)
            msg.plan_progress = plan_progress

        if self.bb.get(IMC_STATE_BB):
            msg.state = self.bb.get(IMC_STATE_BB)
        else:
            currently_running = self.bb.get(CURRENTLY_RUNNING_ACTION)
            if currently_running == 'A_ExecutePlanAction':
                msg.state = IMC_STATE_EXECUTING
            elif currently_running == 'A_EmergencySurface':
                msg.state = IMC_STATE_BLOCKED
                msg.plan_id = 'SAFETY FALLBACK'
                msg.man_id = 'EMERGENCY SURFACE'
                msg.plan_progress = 0.0
            else:
                msg.state = IMC_STATE_READY

        # send message to neptus
        self.plan_control_state_pub.publish(msg)


    def update(self):
        """
        mostly scavenged from Chris's stuff (sprague@kth.se)
        """
        self.update_estimated_state()
        self.update_plan_control_state()
        self.update_vehicle_state()

        return pt.Status.SUCCESS



##########################################################################################
# NOT IMPLEMENTED YET
##########################################################################################

class A_SetManualWaypoint(pt.behaviour.Behaviour):
    def __init__(self):
        #TODO implement to allow for manual single waypoints to be
        # processed by the tree, without sending a whole new plan

        self.bb = pt.blackboard.Blackboard()
        super(A_SetManualWaypoint, self).__init__("A_SetManualWaypoint")

    def update(self):
        return pt.Status.FAILURE

class A_GotoManualWaypoint(pt.behaviour.Behaviour):
    def __init__(self):
        #TODO implement to allow for manual single waypoints to be
        # processed by the tree, without sending a whole new plan

        self.bb = pt.blackboard.Blackboard()
        super(A_GotoManualWaypoint, self).__init__("A_GotoManualWaypoint")

    def update(self):
        return pt.Status.FAILURE
