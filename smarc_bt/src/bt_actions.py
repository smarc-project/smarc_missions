#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import py_trees as pt
import py_trees_ros as ptr

import time
import math
import numpy as np

import rospy
import tf
import actionlib

from smarc_msgs.msg import GotoWaypointAction, GotoWaypointGoal, FloatStamped, GotoWaypoint
from smarc_msgs.srv import UTMToLatLon
import actionlib_msgs.msg as actionlib_msgs
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Header, Bool, Empty
from visualization_msgs.msg import MarkerArray
from geographic_msgs.msg import GeoPoint
# from sensor_msgs.msg import NavSatFix

from std_srvs.srv import SetBool

# from imc_ros_bridge.msg import EstimatedState, VehicleState, PlanDB, PlanDBInformation, PlanDBState, PlanControlState, PlanControl, PlanSpecification, Maneuver

import bb_enums
import imc_enums
import common_globals

from mission_plan import MissionPlan, Waypoint
from mission_log import MissionLog


class A_ReadWaypoint(pt.behaviour.Behaviour):
    def __init__(self,
                 ps_topic,
                 bb_key,
                 utm_to_lat_lon_service_name,
                 reset = False):
        """
        subs to a GotoWaypoint topic and read it into the given bb variable
        """
        super(A_ReadWaypoint, self).__init__(name="A_ReadWaypoint")

        self.bb = pt.blackboard.Blackboard()
        self.ps_topic = ps_topic
        self.last_read_ps = None
        self.last_read_time = None
        self.bb_key = bb_key
        self.utm_to_lat_lon_service_name = utm_to_lat_lon_service_name
        self.got_service = False
        self.reset = reset


    def setup(self, timeout):
        self.ps_sub = rospy.Subscriber(self.ps_topic, GotoWaypoint, self.cb)
        try:
            rospy.loginfo("Waiting for utm to latlon service")
            rospy.wait_for_service(self.utm_to_lat_lon_service_name, timeout=timeout)
            self.got_service = True
        except:
            rospy.logwarn("Could not connect to {}, live WPs wont be updated in the map".format(self.utm_to_lat_lon_service_name))
        return True

    def cb(self, msg):
        self.last_read_ps = msg
        self.last_read_time = time.time()

    def update(self):
        if self.last_read_time is not None:
            time_since = time.time() - self.last_read_time
            self.feedback_message = "Last read:{:.2f}s ago".format(time_since)
        else:
            self.feedback_message = "No msg rcvd"


        if self.last_read_ps is not None:
            pos = self.last_read_ps.pose.pose.position

            if self.last_read_ps.speed_control_mode == GotoWaypoint.SPEED_CONTROL_RPM:
                speed = self.last_read_ps.travel_rpm
            else:
                speed = self.last_read_ps.travel_speed


            lat = 0
            lon = 0
            if self.got_service:
                try:
                    serv = rospy.ServiceProxy(self.utm_to_lat_lon_service_name, UTMToLatLon)
                    p = Point()
                    p.x = pos.x
                    p.y = pos.y
                    p.z = pos.z
                    res = serv(p)
                    lat = res.lat_lon_point.latitude
                    lon = res.lat_lon_point.longitude
                except Exception as e:
                    print(e)

            wp = Waypoint(
                lat = lat,
                lon = lon,
                maneuver_id = 'unplanned_goto',
                maneuver_imc_id = imc_enums.MANEUVER_GOTO,
                maneuver_name = 'unplanned_goto',
                x = pos.x,
                y = pos.y,
                z = pos.z,
                z_unit = self.last_read_ps.z_control_mode,
                speed = speed,
                speed_unit = self.last_read_ps.speed_control_mode,
                tf_frame = self.last_read_ps.pose.header.frame_id,
                extra_data = None)

            if wp.tf_frame != 'utm':
                rospy.logwarn("Unplanned WP is not in UTM frame? It is:{}".format(wp.tf_frame))

            self.bb.set(self.bb_key, wp)


        if self.reset:
            self.last_read_ps = None
            self.bb.set(self.bb_key, None)

        return pt.Status.SUCCESS




class A_ReadLolo(pt.behaviour.Behaviour):
    def __init__(self,
                 robot_name,
                 elevator_topic,
                 elevon_port_topic,
                 elevon_strb_topic,
                 aft_tank_topic,
                 front_tank_topic):
        super(A_ReadLolo, self).__init__(name="A_ReadLolo")
        self.bb = pt.blackboard.Blackboard()
        self.robot_name = robot_name
        self.elevator_topic = elevator_topic
        self.elevon_port_topic = elevon_port_topic
        self.elevon_strb_topic = elevon_strb_topic
        self.aft_tank_topic = aft_tank_topic
        self.front_tank_topic = front_tank_topic

        self.elevator_sub = None
        self.elevon_port_sub = None
        self.elevon_strb_sub = None
        self.aft_tank_sub = None
        self.front_tank_sub = None

        self.elevator = None
        self.elevon_port = None
        self.elevon_strb = None
        self.aft_tank = None
        self.aft_tank_target = None
        self.front_tank = None
        self.front_tank_target = None

        self.anim_frames = ['^','<','v','>']
        self.elev_anim_frame = 0
        self.elevonp_anim_frame = 0
        self.elevons_anim_frame = 0
        self.aft_anim_frame = 0
        self.front_anim_frame = 0

        self.disabled = False

    def elev_cb(self, msg):
        self.elevator = msg.data
        self.elev_anim_frame = (self.elev_anim_frame + 1) %(len(self.anim_frames))
    def elevon_port_cb(self, msg):
        self.elevon_port = msg.data
        self.elevonp_anim_frame = (self.elevonp_anim_frame + 1) %(len(self.anim_frames))
    def elevon_strb_cb(self, msg):
        self.elevon_strb = msg.data
        self.elevons_anim_frame = (self.elevons_anim_frame + 1) %(len(self.anim_frames))
    def aft_tank_cb(self, msg):
        self.aft_tank = msg.percent_current
        self.aft_tank_target = msg.percent_target
        self.aft_anim_frame = (self.aft_anim_frame + 1) %(len(self.anim_frames))
    def front_tank_cb(self, msg):
        self.front_tank = msg.percent_current
        self.front_tank_target = msg.percent_target
        self.front_anim_frame = (self.front_anim_frame + 1) %(len(self.anim_frames))


    def setup(self, timeout):
        if 'lolo' not in self.robot_name:
            self.disabled = True
            return True
        from lolo_msgs.msg import VbsTank
        self.elevator_sub = rospy.Subscriber(self.elevator_topic, FloatStamped, self.elev_cb)
        self.elevon_port_sub = rospy.Subscriber(self.elevon_port_topic, FloatStamped, self.elevon_port_cb)
        self.elevon_strb_sub = rospy.Subscriber(self.elevon_strb_topic, FloatStamped, self.elevon_strb_cb)
        self.aft_tank_sub = rospy.Subscriber(self.aft_tank_topic, VbsTank, self.aft_tank_cb)
        self.front_tank_sub = rospy.Subscriber(self.front_tank_topic, VbsTank, self.front_tank_cb)
        return True

    def update(self):
        if self.disabled:
            self.feedback_message = "Robot not LOLO"
            return pt.Status.SUCCESS

        self.bb.set(bb_enums.LOLO_ELEVATOR, self.elevator)
        self.bb.set(bb_enums.LOLO_ELEVON_PORT, self.elevon_port)
        self.bb.set(bb_enums.LOLO_ELEVON_STRB, self.elevon_strb)
        self.bb.set(bb_enums.LOLO_AFT_TANK, self.aft_tank)
        self.bb.set(bb_enums.LOLO_AFT_TANK_TARGET, self.aft_tank_target)
        self.bb.set(bb_enums.LOLO_FRONT_TANK, self.front_tank)
        self.bb.set(bb_enums.LOLO_FRONT_TANK_TARGET, self.front_tank_target)


        frame = 'elv{} enp{} ens{} aft{} frn{}'.format(
            self.anim_frames[self.elev_anim_frame],
            self.anim_frames[self.elevonp_anim_frame],
            self.anim_frames[self.elevons_anim_frame],
            self.anim_frames[self.aft_anim_frame],
            self.anim_frames[self.front_anim_frame])
        self.feedback_message = frame
        return pt.Status.SUCCESS



class A_PublishFinalize(pt.behaviour.Behaviour):
    def __init__(self, topic):
        super(A_PublishFinalize, self).__init__(name="A_PublishFinalize")
        self.bb = pt.blackboard.Blackboard()
        self.topic = topic

        self.last_published_time = None

        self.message_object = Empty()


    def setup(self, timeout):
        self.pub = rospy.Publisher(self.topic, Empty, queue_size=1)
        return True


    def update(self):
        if self.last_published_time is not None:
            time_since = time.time() - self.last_published_time
            self.feedback_message = "Last pub'd:{:.2f}s ago".format(time_since)
        else:
            self.feedback_message = "Never published!"

        finalized = self.bb.get(bb_enums.MISSION_FINALIZED)
        if not finalized:
            try:
                self.pub.publish(self.message_object)
                self.last_published_time = time.time()
                self.bb.set(bb_enums.MISSION_FINALIZED, True)
                mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
                mission_plan.plan_is_go = False
                self.feedback_message = "Mission finalized, plan is go<-False"
                return pt.Status.SUCCESS
            except:
                msg = "Couldn't publish"
                rospy.logwarn_throttle(1, msg)
                self.feedback_message = msg
                return pt.Status.FAILURE

        return pt.Status.SUCCESS





class A_ManualMissionLog(pt.behaviour.Behaviour):
    def __init__(self, config):
        super(A_ManualMissionLog, self).__init__(name="A_ManualMissionLog")
        self.bb = pt.blackboard.Blackboard()
        self.started_logs = 0
        self.num_saved_logs = 0

        # used just for the latlontoutm function only
        self.mplan = MissionPlan(plandb_msg = None,
                                 auv_config = config,
                                 waypoints = [])


    def start_new_log(self):
        save_location = self.bb.get(bb_enums.MISSION_LOG_FOLDER)
        log = MissionLog(mission_plan = None,
                         robot_name = self.bb.get(bb_enums.ROBOT_NAME),
                         save_location = save_location)
        self.bb.set(bb_enums.MANUAL_MISSION_LOG_OBJ, log)
        rospy.loginfo("Started new manual mission log")
        self.started_logs += 1
        return log

    def update(self):
        enabled = self.bb.get(bb_enums.ENABLE_MANUAL_MISSION_LOG)
        log = self.bb.get(bb_enums.MANUAL_MISSION_LOG_OBJ)

        if not enabled:
            # if we have a log, we save it now
            # and set it to None, so next time we are
            # disabled we dont do anything
            if log is not None:
                log.save()
                self.bb.set(bb_enums.MANUAL_MISSION_LOG_OBJ, None)
                self.num_saved_logs += 1

            self.feedback_message = "Disabled, {} logs saved".format(self.num_saved_logs)
            return pt.Status.SUCCESS


        if log is None:
            log = self.start_new_log()

        # check if there is already a mission plan
        mplan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        # otherwise use our default mplan
        if mplan is None:
            mplan = self.mplan

        log.log(bb = self.bb,
                mplan = mplan,
                t = rospy.get_time())

        self.feedback_message = "Log len:{} of log#{}".format(len(log.navigation_trace), self.started_logs)

        return pt.Status.SUCCESS






class A_SaveMissionLog(pt.behaviour.Behaviour):
    def __init__(self):
        super(A_SaveMissionLog, self).__init__(name="A_SaveMissionLog")
        self.bb = pt.blackboard.Blackboard()
        self.num_saved_logs = 0


    def update(self):
        log = self.bb.get(bb_enums.MISSION_LOG_OBJ)
        if log is not None:
            log.save()
            self.num_saved_logs += 1
            self.bb.set(bb_enums.MISSION_LOG_OBJ, None)
            self.feedback_message = "Saved log #{}!".format(self.num_saved_logs)
        else:
            self.feedback_message = "#saved logs:{}".format(self.num_saved_logs)

        return pt.Status.SUCCESS


class A_UpdateMissionLog(pt.behaviour.Behaviour):
    def __init__(self):
        super(A_UpdateMissionLog, self).__init__(name="A_UpdateMissionLog")
        self.bb = pt.blackboard.Blackboard()
        self.started_logs = 0


    def start_new_log(self, mplan):
        save_location = self.bb.get(bb_enums.MISSION_LOG_FOLDER)
        log = MissionLog(mission_plan = mplan,
                         robot_name = self.bb.get(bb_enums.ROBOT_NAME),
                         save_location = save_location)
        self.bb.set(bb_enums.MISSION_LOG_OBJ, log)
        rospy.loginfo("Started new mission log {}".format(log.data_full_path))
        self.started_logs += 1
        return log


    def update(self):
        # only update if there is an unfinalized mission that has been started
        mplan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mplan is None:
            self.feedback_message = "No mission plan!"
            return pt.Status.FAILURE


        log = self.bb.get(bb_enums.MISSION_LOG_OBJ)
        if log is None:
            log = self.start_new_log(mplan)


        # check if the mission has changed in the meantime
        # this can happen when the user starts a mission, stops it,
        # and then starts a different one
        # we dont wanna log the incomplete one
        # did it change since we last got called?
        if abs(log.creation_time - mplan.creation_time) > 5:
            # it changed!
            # re-start a log
            print("Log was old...")
            log = self.start_new_log(mplan)


        log.log(bb = self.bb,
                mplan = mplan,
                t = rospy.get_time())

        self.feedback_message = "Log len:{} of log#{}".format(len(log.navigation_trace), self.started_logs)

        return pt.Status.SUCCESS








class A_SetDVLRunning(pt.behaviour.Behaviour):
    def __init__(self, dvl_on_off_service_name, running, cooldown):
        super(A_SetDVLRunning, self).__init__(name="A_SetDVLRunning")
        self.switcher_service = rospy.ServiceProxy(dvl_on_off_service_name,
                                                   SetBool)
        self.bb = pt.blackboard.Blackboard()

        self.sb = SetBool()
        self.sb.data = running
        self.running = running

        self.last_toggle = 0
        self.cooldown = cooldown

        self.service_name = dvl_on_off_service_name

    def update(self):
        # try not to call the service every tick...
        dvl_is_running = self.bb.get(bb_enums.DVL_IS_RUNNING)
        if dvl_is_running is not None:
            if dvl_is_running == self.sb.data:
                rospy.loginfo_throttle_identical(20, "DVL is already running:"+str(self.sb.data))
                return pt.Status.SUCCESS

        # check if enough time has passed since last call
        t = time.time()
        if t - self.last_toggle < self.cooldown:
            # nope, return running while we wait
            rospy.loginfo_throttle_identical(5, "Waiting on DVL toggle cooldown")
            return pt.Status.RUNNING

        try:
            ret = self.switcher_service(self.running)
        except rospy.service.ServiceException:
            rospy.logwarn_throttle_identical(60, "DVL Start/stop service not found! Succeeding by default namespace:{}".format(self.service_name))
            return pt.Status.SUCCESS

        if ret.success:
            rospy.loginfo_throttle_identical(5, "DVL TOGGLED:"+str(self.sb.data))
            self.last_toggle = time.time()
            self.bb.set(bb_enums.DVL_IS_RUNNING, self.sb.data)
            return pt.Status.SUCCESS

        rospy.logwarn_throttle_identical(5, "DVL COULD NOT BE TOGGLED:{}, ret:{}".format(self.sb.data, ret))
        return pt.Status.FAILURE



class A_SetNextPlanAction(pt.behaviour.Behaviour):
    def __init__(self, do_not_visit=False):
        """
        Sets the current plan action to the next one
        SUCCESS if it can set it to something that is not None
        FAILURE otherwise

        if do_not_visit=True, then this action will only get the current wp
        and set it and wont actually advance the plan forward.
        This is useful for when you want to set the current wp right after
        you created a plan.
        """
        self.bb = pt.blackboard.Blackboard()
        super(A_SetNextPlanAction, self).__init__('A_SetNextPlanAction')
        self.do_not_visit = do_not_visit

    def update(self):
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            rospy.logwarn_throttle(5, "Mission plan was None!")
            return pt.Status.FAILURE

        if not self.do_not_visit:
            mission_plan.visit_wp()

        next_action = mission_plan.get_current_wp()
        if next_action is None:
            self.feedback_message = "Next action was None"
            rospy.logwarn_throttle(5, "Mission is complete:{}".format(mission_plan.is_complete()))
            return pt.Status.FAILURE

        rospy.loginfo_throttle_identical(5, "Set CURRENT_PLAN_ACTION {} to: {}".format(self.do_not_visit, str(next_action)))
        self.bb.set(bb_enums.CURRENT_PLAN_ACTION, next_action)
        return pt.Status.SUCCESS




class A_GotoWaypoint(ptr.actions.ActionClient):
    def __init__(self,
                 auv_config,
                 node_name = "A_GotoWaypoint",
                 wp_from_bb = None,
                 live_mode_enabled = False,
                 goalless = False):
        """
        Runs an action server that will move the robot to the given waypoint

        wp_from_bb -> if given, the waypoint will be taken from the given bb variable
        live_mode_enabled -> if True, the waypoint will be re-submitted every tick to the server, wp_from_bb must be given
        goalless -> if True, only an empty goal will be sent to the sever, useful as a "signal to start"
        """

        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)
        self.node_name = node_name

        list_of_maneuvers = self.bb.get(bb_enums.MANEUVER_ACTIONS)
        if list_of_maneuvers is None:
            list_of_maneuvers = [self.node_name]
        else:
            list_of_maneuvers.append(self.node_name)
        self.bb.set(bb_enums.MANEUVER_ACTIONS, list_of_maneuvers)

        self.action_goal_handle = None

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name = self.node_name,
            action_spec = GotoWaypointAction,
            action_goal = None,
            action_namespace = auv_config.ACTION_NAMESPACE,
            override_feedback_message_on_running = "Moving to waypoint"
        )

        self.action_server_ok = False

        self.goal_tf_frame = auv_config.UTM_LINK

        # if given, the action will read a wp from the bb instead of
        # from the mission plan
        self.wp_from_bb = wp_from_bb

        # If true, the action will re-send a goal every time it changes
        # without stopping the action
        self.live_mode_enabled = live_mode_enabled
        self.last_live_update_time = -1

        # every X seconds, try to reconnect to the action server
        # if the server wasnt up and ready when the BT was started
        self.last_reconnect_attempt_time = 0
        self.reconnect_attempt_period = 5

        # make sure there is a bb key given to get the wp from
        # if live mode is needed
        if self.live_mode_enabled and self.wp_from_bb is None:
            assert False, "Live mode must be given a bb key to get the WP from!"

        # if True, will not attempt to fill in a goal, will submit empty goal objects
        self.goalless = goalless


    def setup(self, timeout):
        """
        Overwriting the normal ptr action setup to stop it from failiing the setup step
        and instead handling this failure in the tree.
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
        else:
            self.action_server_ok = True

        return True


    def make_goal_from_wp(self, wp):
        # get the goal tolerance as a dynamic variable from the bb
        goal_tolerance = self.bb.get(bb_enums.WAYPOINT_TOLERANCE)

        # construct the message
        goal = GotoWaypointGoal()
        goal.waypoint.pose.pose.position.x = wp.x
        goal.waypoint.pose.pose.position.y = wp.y
        goal.waypoint.goal_tolerance = goal_tolerance

        # 0=None, 1=Depth, 2=Altitude in the action
        # thankfully these are the same in IMC and in the Action
        # but Action doesnt have 'height'
        if wp.z_unit == imc_enums.Z_HEIGHT:
            goal.waypoint.z_control_mode = imc_enums.Z_NONE
        else:
            goal.waypoint.z_control_mode = wp.z_unit
        goal.waypoint.travel_depth = wp.z

        goal.waypoint.speed_control_mode = wp.speed_unit
        if wp.speed_unit == GotoWaypoint.SPEED_CONTROL_SPEED:
            goal.waypoint.travel_speed = wp.speed
        else:
            goal.waypoint.travel_rpm = int(wp.speed)

        goal.waypoint.pose.header.frame_id = wp.tf_frame

        goal.waypoint.lat = wp.lat
        goal.waypoint.lon = wp.lon

        return goal

    def send_goal(self):
        self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
        self.sent_goal = True
        self.vehicle.last_goto_wp = self.action_goal.waypoint


    def initialise(self):
        if not self.action_server_ok:
            self.feedback_message = "No action server found for A_GotoWaypoint!"
            rospy.logwarn_throttle(5, self.feedback_message)
            return

        # quick exit for the goalless version
        if self.goalless:
            self.feedback_message = "Goalless initialized"
            self.action_goal = GotoWaypointGoal()
            self.sent_goal = False
            return

        # so we can pass a bb variable name to this action
        # and it would go to that wp
        if self.wp_from_bb is not None:
            wp = self.bb.get(self.wp_from_bb)
            rospy.loginfo_throttle_identical(5, "Acquired WP from the BB key:{}".format(self.wp_from_bb))
        # otherwise default into following the mission plan
        else:
            mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
            if mission_plan is None:
                self.feedback_message = "No mission plan found!"
                rospy.logwarn(self.feedback_message)
                return

            wp = mission_plan.get_current_wp()

        if wp is None:
            if self.wp_from_bb is None:
                self.feedback_message = "No wp found to execute! Does the plan have any waypoints that we understand?"
                rospy.loginfo_throttle(3, self.feedback_message)
            else:
                self.feedback_message = "Unplanned waypoint not found but it is enabled!"
                rospy.loginfo_throttle(3, self.feedback_message)
            return

        if wp.tf_frame != self.goal_tf_frame:
            self.feedback_message = 'The frame of the waypoint({0}) does not match the expected frame({1}) of the action client!'.format(frame, self.goal_tf_frame)
            rospy.logerr_throttle(5, self.feedback_message)
            return

        if wp.maneuver_imc_id != imc_enums.MANEUVER_GOTO:
            rospy.loginfo("THIS IS A GOTO MANEUVER, WE ARE USING IT FOR SOMETHING ELSE")


        self.action_goal = self.make_goal_from_wp(wp)
        rospy.loginfo("Goto waypoint goal initialized:"+str(self.action_goal))
        # ensure that we still need to send the goal
        self.sent_goal = False




    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """

        if not self.action_server_ok:
            self.feedback_message = "Action Server not available!"
            rospy.logerr_throttle_identical(5, self.feedback_message)
            t = time.time()
            diff = t - self.last_reconnect_attempt_time
            if diff < self.reconnect_attempt_period:
                self.feedback_message = "Re-trying to connect in {}s".format(diff)
            else:
                self.setup(self.reconnect_attempt_period-1)

            return pt.Status.FAILURE

        # if your action client is not valid
        if not self.action_client:
            self.feedback_message = "ActionClient is invalid! Client:"+str(self.action_client)
            rospy.logerr(self.feedback_message)
            return pt.Status.FAILURE

        # if the action_goal is invalid
        if not self.action_goal:
            self.feedback_message = "No action_goal!"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if goal hasn't been sent yet
        if not self.sent_goal:
            self.send_goal()
            rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
            self.feedback_message = "Goal sent"
            return pt.Status.RUNNING

        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            self.feedback_message = "Aborted goal"
            rospy.loginfo(self.feedback_message)
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result is not None and result.reached_waypoint:
            self.feedback_message = "Completed goal"
            rospy.loginfo(self.feedback_message)
            return pt.Status.SUCCESS


        # if we are in live mode, re-make a goal and send it again
        if self.live_mode_enabled:
            wp = self.bb.get(self.wp_from_bb)
            if wp is None:
                self.feedback_message = "wp in {} was reset while running!".format(self.wp_from_bb)
                rospy.loginfo_throttle(3, self.feedback_message)
                return pt.Status.FAILURE

            # make sure it is not the exact same wp
            # before making and sending a goal
            goal_pos = self.action_goal.waypoint.pose.pose.position
            xdiff = abs(goal_pos.x - wp.x)
            ydiff = abs(goal_pos.y - wp.y)
            goal_z = self.action_goal.waypoint.travel_depth
            zdiff = abs(goal_z - wp.z)
            # if there is sufficient change in the wp from the previous one
            # update the goal
            # XXX maybe make the 0.5s a dynamic reconfig thing?
            if any([xdiff > 0.5, ydiff > 0.5, zdiff > 0.5]):
                self.action_goal = self.make_goal_from_wp(wp)
                self.send_goal()
                self.last_live_update_time = time.time()
                self.feedback_message = "Sent goal just now"
            else:
                self.feedback_message = "Live updated {:.2f}s ago".format(time.time() - self.last_live_update_time)

        else:
            if self.goalless:
                self.feedback_message = "Running goalless"
            else:
                # no live updates, just report distance to planned wp
                # still running, set our feedback message to distance left
                current_loc = self.vehicle.position_utm
                mplan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
                if mplan is not None and current_loc is not None:
                    wp = mplan.get_current_wp()
                    x,y = current_loc
                    h_dist = math.sqrt( (x-wp.x)**2 + (y-wp.y)**2 )
                    h_dist -= self.bb.get(bb_enums.WAYPOINT_TOLERANCE)
                    v_dist = wp.z - self.vehicle.depth
                    self.feedback_message = "HDist:{:.2f}, VDist:{:.2f} towards {}".format(h_dist, v_dist, wp.maneuver_name)

        return pt.Status.RUNNING


    def feedback_cb(self, msg):
        fb = str(msg.ETA)
        rospy.loginfo_throttle(5, "feedback from server:{}".format(fb))



class A_PublishMissionPlan(pt.behaviour.Behaviour):
    """
    Publishes the current plans waypoints as a PoseArray
    """
    def __init__(self, plan_viz_topic, plan_path_topic):
        super(A_PublishMissionPlan, self).__init__(name="A_PublishMissionPlan")
        self.bb = pt.blackboard.Blackboard()
        self.pa_pub = None
        self.plan_viz_topic = plan_viz_topic
        self.plan_path_topic = plan_path_topic

    def setup(self, timeout):
        self.pa_pub = rospy.Publisher(self.plan_viz_topic, PoseArray, queue_size=1)
        self.pp_pub = rospy.Publisher(self.plan_path_topic, Path, queue_size=1)
        return True


    def update(self):
        mission = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission is not None:
            # we wanna flip z's for rviz because it wants HEIGHT not DEPTH
            pa = mission.get_pose_array(flip_z=True)
        else:
            pa = PoseArray()

        self.pa_pub.publish(pa)

        if mission is not None:
            # we dont wanna flip z's into height for nacho
            pa = mission.get_pose_array(flip_z=False)
            pp = Path()
            pp.header.frame_id = mission.plan_frame
            for pose in pa.poses:
                ps = PoseStamped()
                ps.pose = pose
                ps.header.frame_id = mission.plan_frame
                pp.poses.append(ps)
        else:
            pp = Path()

        self.pp_pub.publish(pp)




        return pt.Status.SUCCESS


class A_FollowLeader(ptr.actions.ActionClient):
    def __init__(self,
                 action_namespace,
                 leader_link):
        """
        Runs an action server that will move the robot towards another tf link
        """

        self.bb = pt.blackboard.Blackboard()
        list_of_maneuvers = self.bb.get(bb_enums.MANEUVER_ACTIONS)
        if list_of_maneuvers is None:
            list_of_maneuvers = ["A_FollowLeader"]
        else:
            list_of_maneuvers.append("A_FollowLeader")
        self.bb.set(bb_enums.MANEUVER_ACTIONS, list_of_maneuvers)

        self.action_goal_handle = None
        self.leader_link = leader_link

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name="A_FollowLeader",
            action_spec=GotoWaypointAction,
            action_goal=None,
            action_namespace = action_namespace,
            override_feedback_message_on_running="Moving towards"+str(leader_link)
        )

        self.action_server_ok = False

    def setup(self, timeout):
        """
        Overwriting the normal ptr action setup to stop it from failiing the setup step
        and instead handling this failure in the tree.
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client = actionlib.SimpleActionClient(
            self.action_namespace,
            self.action_spec
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
        else:
            self.action_server_ok = True

        return True


    def initialise(self):
        # construct the message
        self.action_goal = GotoWaypointGoal()
        # leave 0,0,0 because we want to go to the frame's center
        self.action_goal.target_pose.header.frame_id = self.leader_link
        rospy.loginfo("Follow action goal initialized")

        # ensure that we still need to send the goal
        self.sent_goal = False

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """
        if not self.action_server_ok:
            self.feedback_message = "Action Server for follow leader action can not be used!"
            rospy.logerr_throttle_identical(5,self.feedback_message)
            return pt.Status.FAILURE

        # if your action client is not valid
        if not self.action_client:
            self.feedback_message = "ActionClient is invalid! Client:"+str(self.action_client)
            rospy.logerr(self.feedback_message)
            return pt.Status.FAILURE

        # if the action_goal is invalid
        if not self.action_goal:
            self.feedback_message = "No action_goal!"
            rospy.logwarn(self.feedback_message)
            return pt.Status.FAILURE

        # if goal hasn't been sent yet
        if not self.sent_goal:
            self.action_goal_handle = self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
            rospy.loginfo("Sent goal to action server:"+str(self.action_goal))
            self.feedback_message = "Goal sent"
            return pt.Status.RUNNING


        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            self.feedback_message = "Aborted goal"
            rospy.loginfo(self.feedback_message)
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result:
            self.feedback_message = "Completed goal"
            rospy.loginfo(self.feedback_message)
            return pt.Status.SUCCESS


        return pt.Status.RUNNING

    def feedback_cb(self, msg):
        pass


class A_ReadBuoys(pt.behaviour.Behaviour):

    '''
    This action reads the uncertain positions
    (mean and covariance) of buoys from the rostopic.
    '''

    def __init__(
        self,
        topic_name,
        buoy_link,
        utm_link,
        latlon_utm_serv,
    ):

        # rostopic name and type (e.g. marker array)
        self.topic_name = topic_name

        # frame IDs for TF
        self.buoy_link = buoy_link
        self.utm_link = utm_link

        # lat/lon to utm service
        self.latlon_utm_serv = latlon_utm_serv

        # blackboard for info
        self.bb = pt.blackboard.Blackboard()

        # become a behaviour
        pt.behaviour.Behaviour.__init__(
            self,
            name="A_ReadBuoys"
        )

        # for coordinate frame transformations
        self.tf_listener = tf.TransformListener()

    def setup(self, timeout):

        # wait for TF transformation
        try:
            rospy.loginfo('Waiting for transform from {} to {}.'.format(
                self.buoy_link,
                self.utm_link
            ))
            self.tf_listener.waitForTransform(
                self.buoy_link,
                self.utm_link,
                rospy.Time(),
                rospy.Duration(timeout)
            )
        except:
            rospy.loginfo('Transform from {} to {} not found.'.format(
                self.buoy_link,
                self.utm_link
            ))

        # subscribe to buoy positions
        self.sub = rospy.Subscriber(
            self.topic_name,
            MarkerArray,
            callback=self.cb,
            queue_size=10
        )
        # self.bb.set(bb_enums.BUOYS, None)
        self.buoys = None
        return True

    def cb(self, msg):

        '''
        This will read the uncertain buoy positions
        from the SLAM backend and sensors.
        But, for now, it just read the simulator buoys.
        The buoys here are assumed to be in the map frame.
        '''

        # space for bouy positions
        # rospy.loginfo('hello')
        self.buoys = list()

        # loop through visualization markers
        for marker in msg.markers:

            # convert their pose to pose stamped
            pose = PoseStamped(
                header=marker.header,
                pose=marker.pose
            )

            # # transform it from local to UTM frame
            # pose = self.tf_listener.transformPose(
            #     self.utm_link,
            #     pose
            # )

            # add it to the list
            self.buoys.append([
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z
            ])

        # make it into a numpy array because why not
        self.buoys = np.array(self.buoys)
        self.buoys = self.buoys[np.argsort(self.buoys[:,0])]
        self.buoys = self.buoys.reshape((-1, 3, 3))
        self.buoys = np.sort(self.buoys, axis=1)
        self.buoys = dict(
            front=self.buoys[:,0,:],
            left=self.buoys[0,:,:],
            back=self.buoys[:,-1,:],
            right=self.buoys[-1,:,:],
            all=self.buoys
        )

    def update(self):

        # put the buoy positions in the blackboard
        self.bb.set(bb_enums.BUOYS, self.buoys)
        return pt.Status.SUCCESS
