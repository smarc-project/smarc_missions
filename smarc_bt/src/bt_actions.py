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
from smarc_msgs.srv import UTMToLatLon, LatLonToUTM
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
                 lat_lon_to_utm_service_name,
                 reset = False):
        """
        subs to a GotoWaypoint topic and read it into the given bb variable
        """
        super(A_ReadWaypoint, self).__init__(name="A_ReadWaypoint")

        self.bb = pt.blackboard.Blackboard()
        self.ps_topic = ps_topic
        self.last_read_wp = None
        self.last_read_time = None
        self.bb_key = bb_key
        self.utm_to_lat_lon_service_name = utm_to_lat_lon_service_name
        self.lat_lon_to_utm_service_name = lat_lon_to_utm_service_name
        self.got_utm_service = False
        self.got_latlon_service = False
        self.reset = reset


    def setup(self, timeout):
        self.ps_sub = rospy.Subscriber(self.ps_topic, GotoWaypoint, self.cb)
        try:
            rospy.loginfo("Waiting for utm to latlon service")
            rospy.wait_for_service(self.utm_to_lat_lon_service_name, timeout=timeout)
            self.got_utm_service = True
        except:
            rospy.logwarn("Could not connect to {}, live WPs wont be updated in the map".format(self.utm_to_lat_lon_service_name))

        try:
            rospy.loginfo("Waiting for latlon to utm service")
            rospy.wait_for_service(self.lat_lon_to_utm_service_name, timeout=timeout)
            self.got_latlon_service = True
        except:
            rospy.logwarn("Could not connect to {}, we cant read WPs from a GUI".format(self.lat_lon_to_utm_service_name))
        return True

    def cb(self, msg):
        self.last_read_wp = msg
        self.last_read_time = time.time()

    def update(self):
        if self.last_read_time is not None:
            time_since = time.time() - self.last_read_time
            self.feedback_message = "Last read:{:.2f}s ago".format(time_since)
        else:
            self.feedback_message = "No msg rcvd"

        if self.last_read_wp is None:
            return pt.Status.SUCCESS

        wp = Waypoint(goto_waypoint = self.last_read_wp,
                      imc_man_id = imc_enums.MANEUVER_GOTO)


        frame_id = self.last_read_wp.pose.header.frame_id

        # given a latlon point, convert to utm for the controllers
        if frame_id == "latlon":
            if not self.got_latlon_service:
                self.feedback_message = "Given a latlon point but got no service!"
                return pt.Status.FAILURE
            try:
                serv = rospy.ServiceProxy(self.lat_lon_to_utm_service_name, LatLonToUTM)
                wp.set_utm_from_latlon(serv)
            except Exception as e:
                print(e)
                return pt.Status.FAILURE


        # given a utm point, convert to latlon for any guis
        if frame_id == 'utm':
            if self.got_utm_service:
                try:
                    serv = rospy.ServiceProxy(self.utm_to_lat_lon_service_name, UTMToLatLon)
                    wp.set_latlon_from_utm(serv)
                except Exception as e:
                    print(e)

        wp.wp.pose.header.frame_id = 'utm'


        if not wp.is_actionable:
            self.feedback_message = "Empty wp!"
            return pt.Status.SUCCESS


        self.bb.set(self.bb_key, wp)


        if self.reset:
            self.last_read_wp = None
            self.bb.set(self.bb_key, None)

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
        self.mplan = MissionPlan(auv_config = config,
                                 mission_control_msg = None,
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
                 action_namespace = None,
                 node_name = "A_GotoWaypoint",
                 wp_from_bb = None,
                 live_mode_enabled = False,
                 goalless = False):
        """
        Runs an action server that will move the robot to the given waypoint

        action_namespace -> if given, will send the goal to that server instead of
        the default goto_waypoint
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

        if action_namespace is None:
            action_namespace = auv_config.GOTO_ACTION_NAMESPACE

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name = self.node_name,
            action_spec = GotoWaypointAction,
            action_goal = None,
            action_namespace = action_namespace,
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
        goal.waypoint = wp.wp
        goal.waypoint.goal_tolerance = goal_tolerance

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

        if wp.frame_id != self.goal_tf_frame:
            self.feedback_message = 'The frame of the waypoint({0}) does not match the expected frame({1}) of the action client!'.format(wp.frame_id, self.goal_tf_frame)
            rospy.logerr_throttle(5, self.feedback_message)
            return


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
            if wp.is_too_similar_to_other(self.action_goal.waypoint):
                if self.last_live_update_time < 5:
                    self.feedback_message = "Live updated just now"
                else:
                    self.feedback_message = "Live updated {:.2f}s ago".format(time.time() - self.last_live_update_time)
            else:
                self.action_goal = self.make_goal_from_wp(wp)
                self.send_goal()
                self.last_live_update_time = time.time()
                self.feedback_message = "Sent goal just now"

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
                    v_dist = wp.depth - self.vehicle.depth
                    self.feedback_message = "HDist:{:.2f}, VDist:{:.2f} towards {}".format(h_dist, v_dist, wp.wp.name)

        return pt.Status.RUNNING


    def feedback_cb(self, msg):
        fb = str(msg.ETA)
        rospy.loginfo_throttle(5, "feedback from server:{}".format(fb))

