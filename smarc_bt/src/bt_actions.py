#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

from logging import makeLogRecord
from math import dist
from os import stat
import py_trees as pt
import py_trees_ros as ptr

import time
import numpy as np

import rospy
import tf
import actionlib
from itertools import chain, combinations
from sklearn.mixture import BayesianGaussianMixture
from sklearn.cluster import KMeans
#  from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smarc_msgs.msg import GotoWaypointAction, GotoWaypointGoal
import actionlib_msgs.msg as actionlib_msgs
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Header, Bool, Empty
from tf2_ros import transform_listener
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import NavSatFix
from vision_msgs.msg import Detection2DArray

from std_srvs.srv import SetBool

from imc_ros_bridge.msg import EstimatedState, VehicleState, PlanDB, PlanDBInformation, PlanDBState, PlanControlState, PlanControl, PlanSpecification, Maneuver
import matplotlib.pyplot as plt
import bb_enums
import imc_enums
import common_globals

from mission_plan import MissionPlan, Waypoint



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



class A_EmergencySurface(ptr.actions.ActionClient):
    def __init__(self, emergency_action_namespace):
        """
        What to do when an emergency happens. This should be a very simple
        action that is super unlikely to fail, ever. It should also 'just work'
        without a goal.
        Like surfacing.
        """
        self.bb = pt.blackboard.Blackboard()
        self.action_goal_handle = None

        ptr.actions.ActionClient.__init__(
            self,
            name="A_EmergencySurface",
            action_spec=GotoWaypointAction,
            action_goal=None,
            action_namespace= emergency_action_namespace,
            override_feedback_message_on_running="EMERGENCY SURFACING"
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
            self.action_server_ok = False
        else:
            self.action_server_ok = True

        return True

    def initialise(self):
        if not self.action_server_ok:
            rospy.logwarn_throttle_identical(5, "No Action Server found for emergency action, will just block the tree!")
            return
        self.feedback_message = "EMERGENCY SURFACING"
        # construct the message
        self.action_goal = GotoWaypointGoal()
        self.sent_goal = False

    def update(self):
        if not self.action_server_ok:
            self.feedback_message = "Action Server for emergency action can not be used!"
            rospy.logerr_throttle_identical(5,self.feedback_message)
            return pt.Status.FAILURE

        # if your action client is not valid
        if not self.action_client:
            self.feedback_message = "ActionClient for emergency action is invalid!"
            rospy.logwarn_throttle_identical(5,self.feedback_message)
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
            return pt.Status.RUNNING


        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            self.feedback_message = "Aborted emergency"
            rospy.loginfo(self.feedback_message)
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result:
            self.feedback_message = "Completed emergency"
            rospy.loginfo(self.feedback_message)
            return pt.Status.SUCCESS


        # if we're still trying to accomplish the goal
        return pt.Status.RUNNING

    def feedback_cb(self, msg):
        pass




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
                 action_namespace,
                 goal_tolerance = 1,
                 goal_tf_frame = 'utm',
                 node_name = "A_GotoWaypoint"):
        """
        Runs an action server that will move the robot to the given waypoint
        """

        self.bb = pt.blackboard.Blackboard()
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
            action_namespace = action_namespace,
            override_feedback_message_on_running = "Moving to waypoint"
        )

        self.action_server_ok = False

        assert goal_tolerance >= 1, "Goal tolerance must be >=1!, it is:"+str(goal_tolerance)
        self.goal_tolerance = goal_tolerance

        self.goal_tf_frame = goal_tf_frame


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
        if not self.action_server_ok:
            rospy.logwarn_throttle(5, "No action server found for A_GotoWaypoint!")
            return

        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            rospy.logwarn("No mission plan found!")
            return

        wp = mission_plan.get_current_wp()

        if wp is None:
            rospy.loginfo("No wp found to execute! Does the plan have any waypoints that we understand?")
            return

        if wp.tf_frame != self.goal_tf_frame:
            rospy.logerr_throttle(5, 'The frame of the waypoint({0}) does not match the expected frame({1}) of the action client!'.format(frame, self.goal_tf_frame))
            return

        if wp.maneuver_id != imc_enums.MANEUVER_GOTO:
            rospy.loginfo("THIS IS A GOTO MANEUVER, WE ARE USING IT FOR SOMETHING ELSE")

        # construct the message
        goal = GotoWaypointGoal()
        goal.waypoint_pose.pose.position.x = wp.x
        goal.waypoint_pose.pose.position.y = wp.y
        goal.goal_tolerance = self.goal_tolerance

        # 0=None, 1=Depth, 2=Altitude in the action
        # thankfully these are the same in IMC and in the Action
        # but Action doesnt have 'height'
        if wp.z_unit == imc_enums.Z_HEIGHT:
            wp.z_unit = imc_enums.Z_NONE
        goal.z_control_mode = wp.z_unit
        goal.travel_depth = wp.z

        # 0=None, 1=RPM, 2=speed in the action
        # 0=speed, 1=rpm, 2=percentage in IMC
        if wp.speed_unit == imc_enums.SPEED_UNIT_RPM:
            goal.speed_control_mode = GotoWaypointGoal.SPEED_CONTROL_RPM
            goal.travel_rpm = wp.speed
        elif wp.speed_unit == imc_enums.SPEED_UNIT_MPS:
            goal.speed_control_mode = GotoWaypointGoal.SPEED_CONTROL_SPEED
            goal.travel_speed = wp.speed
        else:
            goal.speed_control_mode = GotoWaypointGoal.SPEED_CONTROL_NONE
            rospy.logwarn_throttle(1, "Speed control of the waypoint action is NONE!")


        self.action_goal = goal

        rospy.loginfo(">>> Goto waypoint action goal initialized:"+str(goal))

        # ensure that we still need to send the goal
        self.sent_goal = False

    def update(self):
        """
        Check only to see whether the underlying action server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.
        """

        if not self.action_server_ok:
            self.feedback_message = "Action Server for gotowp action can not be used!"
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
            self.sent_goal = True
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


        return pt.Status.RUNNING

    def feedback_cb(self, msg):
        fb = str(msg.ETA)
        self.feedback_message = "ETA:"+fb
        rospy.loginfo_throttle(5, fb)



class A_UpdateTF(pt.behaviour.Behaviour):
    def __init__(self, utm_link, base_link):
        """
        reads the current translation and orientation from the TF tree
        and puts that into the BB

        utm_link and base_link are tf link names where utm_link is essentially the world coordinates.
        check the neptus-related actions too for more info on utm_link
        """
        super(A_UpdateTF, self).__init__("A_UpdateTF")
        self.bb = pt.blackboard.Blackboard()
        self.utm_link = utm_link
        self.base_link = base_link
        self.listener = tf.TransformListener()
        self.tf_ok = False

        self.last_read_time = None


    def setup(self, timeout):
        try:
            rospy.loginfo_throttle(3, "Waiting for transform from {} to {}...".format(self.utm_link, self.base_link))
            self.listener.waitForTransform(self.utm_link, self.base_link, rospy.Time(), rospy.Duration(timeout))
            rospy.loginfo_throttle(3, "...Got it")
            self.tf_ok = True
        except:
            rospy.logerr_throttle(5, "Could not find from "+self.utm_link+" to "+self.base_link + "... Nothing except safety will be run")

        return True

    def update(self):
        if self.last_read_time is not None:
            time_since = time.time() - self.last_read_time
            self.feedback_message = "Last read:{:.2f}s ago".format(time_since)
        else:
            self.feedback_message = "No msg received ever"

        try:
            (world_trans, world_rot) = self.listener.lookupTransform(self.utm_link,
                                                                     self.base_link,
                                                                     rospy.Time(0))
            self.last_read_time = time.time()
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr_throttle_identical(5, "Could not get transform between {} and {}".format(self.utm_link, self.base_link))
            return pt.Status.FAILURE
        except:
            rospy.logerr_throttle_identical(5, "Could not do tf lookup for some other reason")
            return pt.Status.FAILURE

        self.bb.set(bb_enums.WORLD_TRANS, world_trans)
        self.bb.set(bb_enums.WORLD_ROT, world_rot)
        # also create this pointstamped object so that we can transform this
        # easily to w/e other frame is needed later
        ps = PointStamped()
        ps.header.frame_id = self.utm_link
        ps.header.stamp = rospy.Time(0)
        ps.point.x = world_trans[0]
        ps.point.y = world_trans[1]
        ps.point.z = world_trans[2]
        self.bb.set(bb_enums.LOCATION_POINT_STAMPED, ps)

        # the Z component is UP, so invert to get "depth"
        self.bb.set(bb_enums.DEPTH, -world_trans[2])


        return pt.Status.SUCCESS




class A_UpdateNeptusPlanControl(pt.behaviour.Behaviour):
    def __init__(self, plan_control_topic):
        super(A_UpdateNeptusPlanControl, self).__init__("A_UpdateNeptusPlanControl")
        self.bb = pt.blackboard.Blackboard()
        self.plan_control_msg = None
        self.plan_control_topic = plan_control_topic

        self.sub = None

    def setup(self, timeout):
        self.sub = rospy.Subscriber(self.plan_control_topic, PlanControl, self.plancontrol_cb)
        return True


    def plancontrol_cb(self, plan_control_msg):
        #  rospy.loginfo("plancontrol_cb {}".format(plan_control_msg))
        self.plan_control_msg = plan_control_msg

    def update(self):
        plan_control_msg = self.plan_control_msg
        if plan_control_msg is None:
            # not receiving anything is ok.
            return pt.Status.SUCCESS

        # check if this message is a 'go' or 'no go' message
        # imc/plan_control(569):
        # int type:[0,1,2,3] req,suc,fail,in prog
        # int op:[0,1,2,3] start, stop, load, get
        # int request_id
        # string plan_id
        # int flags
        # string info

        # the start button in neptus sends:
        # type:0 op:0 plan_id:"string" flags:1
        # stop button sends:
        # type:0 op:1 plan_id:'' flags:1
        # teleop button sends:
        # type:0 op:0 plan_id:"teleoperation-mode" flags:0

        typee = plan_control_msg.type
        op = plan_control_msg.op
        plan_id = plan_control_msg.plan_id
        flags = plan_control_msg.flags

        # somehow this happens...
        if plan_id is None:
            plan_id=''

        # separate well-defined ifs for possible future shenanigans.
        if typee==0 and op==0 and plan_id!='' and flags==1:
            # start button
            # check if the start was given for our current plan
            current_mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
            self.bb.set(bb_enums.PLAN_IS_GO, True)
            self.bb.set(bb_enums.ENABLE_AUTONOMY, False)
            if current_mission_plan is not None and plan_id == current_mission_plan.plan_id:
                rospy.loginfo("Started plan:{}".format(plan_id))
            else:
                if current_mission_plan is None:
                    rospy.logwarn("Start given for plan:{} but we don't have a plan!".format(plan_id))
                else:
                    rospy.logwarn("Start given for plan:{} our plan:{}".format(plan_id, current_mission_plan.plan_id))

        if typee==0 and op==1 and plan_id=='' and flags==1:
            # stop button
            self.bb.set(bb_enums.PLAN_IS_GO, False)
            self.bb.set(bb_enums.ENABLE_AUTONOMY, False)

        # this string is hardcoded in Neptus, so we hardcode it here too!
        if typee==0 and op==0 and plan_id=='teleoperation-mode' and flags==0:
            # teleop button
            self.bb.set(bb_enums.ENABLE_AUTONOMY, True)
            rospy.logwarn_throttle_identical(10, "AUTONOMOUS MODE")

        # reset it until next message
        self.plan_control_msg = None
        return pt.Status.SUCCESS



class A_UpdateNeptusEstimatedState(pt.behaviour.Behaviour):
    def __init__(self,
                 estimated_state_topic,
                 gps_fix_topic,
                 gps_nav_data_topic):
        super(A_UpdateNeptusEstimatedState, self).__init__("A_UpdateNeptusEstimatedState")
        self.bb = pt.blackboard.Blackboard()
        self.estimated_state_pub = None
        self.estimated_state_topic = estimated_state_topic
        self.e_state = EstimatedState()

        self.gps_fix_pub = None
        self.gps_fix_topic = gps_fix_topic
        self.gps_nav_data_pub = None
        self.gps_nav_data_topic = gps_nav_data_topic
        self.gps_fix = NavSatFix()

    def setup(self, timeout):
        self.estimated_state_pub = rospy.Publisher(self.estimated_state_topic, EstimatedState, queue_size=1)
        self.gps_fix_pub = rospy.Publisher(self.gps_fix_topic, NavSatFix, queue_size=1)
        self.gps_nav_data_pub = rospy.Publisher(self.gps_nav_data_topic, NavSatFix, queue_size=1)
        return True


    def update(self):
        lat = self.bb.get(bb_enums.CURRENT_LATITUDE)
        lon = self.bb.get(bb_enums.CURRENT_LONGITUDE)
        depth = self.bb.get(bb_enums.DEPTH)
        world_rot = self.bb.get(bb_enums.WORLD_ROT)

        if depth is None:
            reason = "depth was None, using 0"
            self.feedback_message = reason
            depth = 0

        if lat is None or lon is None or world_rot is None:
            rospy.logwarn_throttle_identical(10, "Could not update neptus estimated state because lat/lon/world_rot was None!")
            return pt.Status.SUCCESS

        # construct message for neptus
        self.e_state.lat = np.radians(lat)
        self.e_state.lon= np.radians(lon)
        self.e_state.depth = depth
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(world_rot)
        self.e_state.psi = np.pi/2. - yaw
        # send the message to neptus
        self.estimated_state_pub.publish(self.e_state)

        # same thing with gps fix
        # the bridge only looks at lat lon height=altitude
        self.gps_fix.latitude = lat
        self.gps_fix.longitude = lon
        self.gps_fix.altitude = -depth
        self.gps_fix.header.seq = int(time.time())
        self.gps_fix_pub.publish(self.gps_fix)
        self.gps_nav_data_pub.publish(self.gps_fix)

        return pt.Status.SUCCESS


class A_UpdateNeptusPlanControlState(pt.behaviour.Behaviour):
    def __init__(self, plan_control_state_topic):
        super(A_UpdateNeptusPlanControlState, self).__init__("A_UpdateNeptusPlanControlState")
        self.bb = pt.blackboard.Blackboard()
        self.plan_control_state_pub = None
        self.plan_control_state_topic = plan_control_state_topic


    def setup(self, timeout):
        self.plan_control_state_pub = rospy.Publisher(self.plan_control_state_topic, PlanControlState, queue_size=1)
        return True


    def update(self):
        # construct current progress message for neptus
        msg = PlanControlState()
        tip_name = self.bb.get(bb_enums.TREE_TIP_NAME)
        tip_status = self.bb.get(bb_enums.TREE_TIP_STATUS)

        # this tip_status looks like: "Status.FAILURE"
        # I just wanna get the first letter after dot.
        msg.man_id = tip_name+'('+tip_status[7]+')'

        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            msg.plan_id = 'No plan'
            msg.plan_progress = 100.0
        elif mission_plan.is_complete():
            msg.plan_id = 'Mission complete'
            msg.plan_progress = 100.0
        else:
            current_wp_index = mission_plan.current_wp_index
            current_man_id = mission_plan.waypoint_man_ids[current_wp_index]
            total = len(mission_plan.waypoints)
            msg.plan_id = str(mission_plan.plan_id)
            if self.bb.get(bb_enums.PLAN_IS_GO):
                msg.man_id = current_man_id

            plan_progress = (current_wp_index * 100.0) / total # percent float
            msg.plan_progress = plan_progress


        if tip_name in imc_enums.EXECUTING_ACTION_NAMES:
            msg.state = imc_enums.STATE_EXECUTING
        elif tip_name in imc_enums.BLOCKED_ACTION_NAMES:
            msg.state = imc_enums.STATE_BLOCKED
            msg.plan_id = 'SAFETY FALLBACK'
            msg.man_id = 'EMERGENCY'
            msg.plan_progress = 0.0
        else:
            msg.state = imc_enums.STATE_READY

        if self.bb.get(bb_enums.ENABLE_AUTONOMY):
            msg.plan_id += '(AUTONOMOUS)'

        # send message to neptus
        self.plan_control_state_pub.publish(msg)
        return pt.Status.SUCCESS


class A_UpdateNeptusVehicleState(pt.behaviour.Behaviour):
    def __init__(self, vehicle_state_topic):
        super(A_UpdateNeptusVehicleState, self).__init__("A_UpdateNeptusVehicleState")
        self.bb = pt.blackboard.Blackboard()
        self.vehicle_state_pub = None
        self.vehicle_state_topic = vehicle_state_topic

    def setup(self, timeout):
        self.vehicle_state_pub = rospy.Publisher(self.vehicle_state_topic, VehicleState, queue_size=1)
        return True


    def update(self):
        """
        this is the message that makes SAM:DISCONNECTED better.
        """
        vs = VehicleState()

        tip_name = self.bb.get(bb_enums.TREE_TIP_NAME)

        if tip_name in imc_enums.EXECUTING_ACTION_NAMES:
            vs.op_mode = imc_enums.OP_MODE_MANEUVER
        elif tip_name == 'A_EmergencySurface':
            vs.op_mode = imc_enums.OP_MODE_ERROR
        else:
            vs.op_mode = imc_enums.OP_MODE_SERVICE

        self.vehicle_state_pub.publish(vs)
        return pt.Status.SUCCESS


class A_UpdateNeptusPlanDB(pt.behaviour.Behaviour):
    def __init__(self,
                 plandb_topic,
                 utm_link,
                 local_link,
                 latlontoutm_service_name,
                 latlontoutm_service_name_alternative):
        super(A_UpdateNeptusPlanDB, self).__init__("A_UpdateNeptusPlanDB")
        self.bb = pt.blackboard.Blackboard()
        # neptus sends lat/lon, which we convert to utm, which we then convert to local
        self.utm_link = utm_link
        self.local_link = local_link
        self.latlontoutm_service_name = latlontoutm_service_name
        self.latlontoutm_service_name_alternative = latlontoutm_service_name_alternative

        # the message body is largely the same, so we can re-use most of it
        self.plandb_msg = PlanDB()
        self.plandb_msg.type = imc_enums.PLANDB_TYPE_SUCCESS
        self.plandb_msg.op = imc_enums.PLANDB_OP_SET


        self.plandb_pub = None
        self.plandb_sub = None
        self.latest_plandb_msg = None
        self.plandb_topic = plandb_topic


    def setup(self, timeout):
        self.plandb_pub = rospy.Publisher(self.plandb_topic, PlanDB, queue_size=1)
        self.plandb_sub = rospy.Subscriber(self.plandb_topic, PlanDB, callback=self.plandb_cb, queue_size=1)
        return True



    def plandb_cb(self, plandb_msg):
        """
        as an answer to OUR answer of 'type=succes, op=set', neptus sends a 'type=request, op=get_info'.
        """
        #  rospy.loginfo("plandb_db {}".format(plandb_msg))
        self.latest_plandb_msg = plandb_msg


    def make_plandb_info(self):
        current_mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        plan_info = PlanDBInformation()
        plan_info.plan_id = current_mission_plan.plan_id
        plan_info.md5 = current_mission_plan.plandb_msg.plan_spec_md5
        plan_info.change_time = current_mission_plan.creation_time/1000.0
        return plan_info


    def handle_request_get_info(self, plandb_msg):
        # we need to respond to this with some info... but what?
        rospy.loginfo_throttle_identical(30, "Got REQUEST GET_INFO planDB msg from Neptus")

        current_mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_mission_plan is None:
            return

        response = PlanDB()
        response.plan_id = current_mission_plan.plan_id
        response.type = imc_enums.PLANDB_TYPE_SUCCESS
        response.op = imc_enums.PLANDB_OP_GET_INFO
        response.plandb_information = self.make_plandb_info()
        self.plandb_pub.publish(response)
        rospy.loginfo_throttle_identical(30, "Answered GET_INFO for plan:"+str(response.plan_id))

    def handle_request_get_state(self, plandb_msg):
        rospy.loginfo_throttle_identical(30, "Got REQUEST GET_STATE planDB msg from Neptus")
        current_mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_mission_plan is None:
            return

        # https://github.com/LSTS/imcjava/blob/d95fddeab4c439e603cf5e30a32979ad7ace5fbc/src/java/pt/lsts/imc/adapter/PlanDbManager.java#L160
        # See above for an example
        # TODO it seems like we need to keep a planDB ourselves on this side, collect all the plans we
        # received and answer this get_state with data from them all.
        # lets try telling neptus that we just got one plan, maybe that'll be okay?
        # seems alright, but after this message is sent, the plan goes red :/
        response = PlanDB()
        response.plan_id = current_mission_plan.plan_id
        response.type = imc_enums.PLANDB_TYPE_SUCCESS
        response.op = imc_enums.PLANDB_OP_GET_STATE

        response.plandb_state = PlanDBState()
        response.plandb_state.plan_count = 1
        response.plandb_state.plans_info.append(self.make_plandb_info())

        self.plandb_pub.publish(response)
        rospy.loginfo_throttle_identical(30, "Answered GET_STATE for plan:\n"+str(response.plan_id))

    def handle_set_plan(self, plandb_msg):
        # there is a plan we can at least look at
        mission_plan = MissionPlan(plan_frame = self.utm_link,
                                   plandb_msg = plandb_msg,
                                   latlontoutm_service_name = self.latlontoutm_service_name,
                                   latlontoutm_service_name_alternative = self.latlontoutm_service_name_alternative,
                                   coverage_swath = self.bb.get(bb_enums.SWATH),
                                   vehicle_localization_error_growth = self.bb.get(bb_enums.LOCALIZATION_ERROR_GROWTH))

        if mission_plan.no_service:
            self.feedback_message = "MISSION PLAN HAS NO SERVICE"
            rospy.logerr(self.feedback_message)
            return

        self.bb.set(bb_enums.MISSION_PLAN_OBJ, mission_plan)
        self.bb.set(bb_enums.ENABLE_AUTONOMY, False)
        self.bb.set(bb_enums.MISSION_FINALIZED, False)
        self.bb.set(bb_enums.PLAN_IS_GO, False)
        rospy.loginfo_throttle_identical(5, "Set the mission plan to:{} and un-finalized the mission.".format(mission_plan))


    def handle_plandb_msg(self):
        plandb_msg = self.latest_plandb_msg
        if plandb_msg is None:
            return

        typee = plandb_msg.type
        op = plandb_msg.op

        # request get_info
        if typee == imc_enums.PLANDB_TYPE_REQUEST and op == imc_enums.PLANDB_OP_GET_INFO:
            self.handle_request_get_info(plandb_msg)

        elif typee == imc_enums.PLANDB_TYPE_REQUEST and op == imc_enums.PLANDB_OP_GET_STATE:
            self.handle_request_get_state(plandb_msg)

        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_SET:
            self.feedback_message =  "Got SUCCESS for plandb set"

        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_GET_INFO:
            self.feedback_message =  "Got SUCCESS for plandb get info"

        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_GET_STATE:
            self.feedback_message =  "Got SUCCESS for plandb get state"

        elif op == imc_enums.PLANDB_OP_SET:
            self.handle_set_plan(plandb_msg)

        else:
            self.feedback_message = "Got some unhandled planDB message:\n"+str(plandb_msg)




    def respond_set_success(self):
        current_mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_mission_plan is None:
            self.feedback_message = "No mission plan obj!"
            return

        plan_id = current_mission_plan.plan_id
        self.plandb_msg.plan_id = plan_id
        self.plandb_pub.publish(self.plandb_msg)
        self.feedback_message =  "Answered set success for plan_id:"+str(plan_id)

    def update(self):
        # we just want to tell neptus we got the plan all the time
        # this keeps the thingy green
        self.respond_set_success()
        self.handle_plandb_msg()
        # reset
        self.latest_plandb_msg = None
        return pt.Status.SUCCESS


class A_UpdateMissonForPOI(pt.behaviour.Behaviour):
    """
    creates a new diamond-shaped mission over a detected POI
    and sets that as the current mission plan.
    always returns SUCCESS
    """
    def __init__(self, utm_link, poi_link, latlontoutm_service_name):
        super(A_UpdateMissonForPOI, self).__init__(name="A_UpdateMissonForPOI")
        self.bb = pt.blackboard.Blackboard()
        self.utm_link = utm_link
        self.poi_link = poi_link
        self.tf_listener = tf.TransformListener()
        self.latlontoutm_service_name = latlontoutm_service_name

        self.poi_link_available = False


    def setup(self, timeout):
        try:
            rospy.loginfo_throttle(3, "Waiting for transform from {} to {}...".format(self.poi_link, self.utm_link))
            self.tf_listener.waitForTransform(self.poi_link, self.utm_link, rospy.Time(), rospy.Duration(timeout))
            rospy.loginfo_throttle(3, "...Got it")
            self.poi_link_available = True
        except:
            rospy.logerr_throttle(5, "Could not find tf from:"+self.poi_link+" to:"+self.utm_link+" disabling updates")

        return True

    def update(self):
        #XXX UNTESTED STUFF HERE, RETURN FAILURE TO KEEP PPL
        #XXX FROM USING THIS ACTION
        return pt.Status.FAILURE

        if not self.poi_link_available:
            return pt.Status.FAILURE

        poi = self.bb.get(bb_enums.POI_POINT_STAMPED)
        if poi is None:
            return pt.Status.SUCCESS
        poi_local = self.tf_listener.transformPoint(self.utm_link, poi)

        x = poi_local.point.x
        y = poi_local.point.y
        depth = poi.point.z

        # construct the waypoints that we want to go to
        inspection_depth = max(1, depth - 5)
        radius = 10
        # go east,west,north,south,center
        # so we do bunch of fly-overs
        waypoints = [
            (x+radius, y, inspection_depth),
            (x-radius, y, inspection_depth),
            (x, y+radius, inspection_depth),
            (x, y-radius, inspection_depth),
            (x, y, 0)
        ]
        waypoint_man_ids = ['east', 'west', 'north', 'south', 'surface_center']
        # construct a planDB message to be given to the mission_plan
        # we will not fill the plan_spec of this plandb message,
        # and instead call a different constructor of MissionPlan
        # to bypass the lat/lon stuff
        pdb = PlanDB()
        pdb.request_id = 42
        pdb.plan_id = "POI"

        # set it in the tree
        mission_plan = MissionPlan(plan_frame = self.utm_link,
                                   plandb_msg = pdb,
                                   waypoints = waypoints,
                                   waypoint_man_ids=waypoint_man_ids,
                                   latlontoutm_service_name = self.latlontoutm_service_name)

        self.bb.set(bb_enums.MISSION_PLAN_OBJ, mission_plan)

        rospy.loginfo_throttle_identical(5, "Due to POI, set the mission plan to:"+str(mission_plan))
        return pt.Status.SUCCESS

class A_VizPublishPlan(pt.behaviour.Behaviour):
    """
    Publishes the current plans waypoints as a PoseArray
    """
    def __init__(self, plan_viz_topic):
        super(A_VizPublishPlan, self).__init__(name="A_VizPublishPlan")
        self.bb = pt.blackboard.Blackboard()
        self.pa_pub = None
        self.plan_viz_topic = plan_viz_topic

    def setup(self, timeout):
        self.pa_pub = rospy.Publisher(self.plan_viz_topic, PoseArray, queue_size=1)
        return True


    def update(self):
        mission = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission is not None:
            pa = mission.get_pose_array(flip_z=True)
        else:
            pa = PoseArray()

        self.pa_pub.publish(pa)


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
            self.sent_goal = True
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
    This action reads the 
    - simulated buoy positions (marker_topic) 
    - and the detected buoy positions (detection_topic)
    '''

    def __init__(
        self, 
        read_markers,
        read_detection, 
        marker_topic, 
        detection_topic,
        heading,
        n_walls,
        atol,
        dtol
    ):

        # toggle
        self.read_markers = read_markers
        self.read_detection = read_detection

        # markers (map frame)
        self.marker_topic = marker_topic

        # detection hypothesis of buoys (odom frame)
        self.detection_topic = detection_topic

        # a priori wall orientation and number of walls
        self.heading = heading
        self.n_walls = n_walls

        # line-fitting angle- and inclusion- tolerances
        self.atol = atol
        self.dtol = dtol

        # blackboard for info
        self.bb = pt.blackboard.Blackboard()

        # become a behaviour
        pt.behaviour.Behaviour.__init__(
            self,
            name="A_ReadBuoys"
        )

    def setup(self, timeout):

        # feedback
        rospy.loginfo('Setting up A_ReadBuoys')

        # initialise variables of interest
        self.bb.set(bb_enums.BUOY_MARKERS, None)
        self.bb.set(bb_enums.BUOY_DETECTION, None)
        self.buoy_markers = None
        self.buoy_detection = None

        # subscribe to buoy markers
        self.sub_markers = rospy.Subscriber(
            self.marker_topic,
            MarkerArray,
            callback=self.get_marker,
            queue_size=10
        )
        
        # # subscribe to buoy detection
        # self.sub_sensor = rospy.Subscriber(
        #     self.detection_topic,
        #     Detection2DArray,
        #     callback=self.get_detection,
        #     queue_size=10
        # )

        return True

    def update(self):

        # if we want to read sim buoys and there're none
        if self.read_markers and self.buoy_markers is None:
            self.feedback_message = 'Sim buoy reader not working.'
            return pt.Status.FAILURE

        # otherwise, put them in the blackboard
        else:
            self.feedback_message = 'Putting sim buoys in blackboard.'
            self.bb.set(bb_enums.BUOY_MARKERS, self.buoy_markers)

        return pt.Status.SUCCESS

    def get_marker(self, markerarray):

        # if we haven't updated the markers
        if self.buoy_markers is None:

            # TF frame
            frame_id = markerarray.markers[0].header.frame_id
            for marker in markerarray.markers:
                assert marker.header.frame_id == frame_id

            # point cloud of buoy positions
            buoys = np.array([[
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ] for marker in markerarray.markers])

            # sort the buoys into walls
            buoys = self.infer_lines(
                points=buoys,
                theta=self.heading,
                n_lines=self.n_walls,
                atol=self.atol,
                dtol=self.dtol
            )

            # put a dictionary into the blackboard
            self.buoy_markers = dict(
                frame_id=frame_id, 
                walls=buoys
            )

        # if we have already recorded them
        else:
            pass

    def get_detection(self, detection2darray):

        # TF frame
        frame_id = detection2darray.detections[0].header.frame_id
        for detection in detection2darray.detections:
            assert detection.header.frame_id == frame_id

        # point cloud of buoy positions
        buoys = list()
        for detection in detection2darray.detections:
            for result in detection.results:
                buoys.append([
                    result.pose.pose.position.x,
                    result.pose.pose.position.y,
                    result.pose.pose.position.z
                ])
        buoys = np.array(buoys)

    @staticmethod
    def infer_lines(points, theta, n_lines, atol, dtol):

        '''
        Clusters a set of partially colinear points
        into a collection of totally colinear points.
        E.g. infer algae walls from a set of buoys
        and a priori known wall angles and number.

        points: (n, >2) np.array of partially colinear points [m]
        theta: float of a priori known angle of colinearity [deg]
        n_walls: int a priori known number of walls
        etol: float error tolerance of wall inference
        '''

        # if we don't have at least 2 points
        if points.shape[0] < 2:
            return list()

        # centroid of points
        centroid = points.mean(axis=0)

        # a priori direction of heading
        theta = np.deg2rad(theta)
        vtheta = np.array([np.cos(theta), np.sin(theta)])

        # all binary subsets of points
        w = np.array([s for s in combinations(points, 2)])

        # Euclidian distance between each subset
        d = w[:,1,:] - w[:,0,:]
        d = np.linalg.norm(d, axis=1)

        # subsets totally ordered
        w = w[np.argsort(d)]

        # maximal walls
        mw = list()

        # loop through subsets in decreasing span-size order
        for s in reversed(w):

            # initialise skip
            skip = False

            # direction of line
            vline = s[-1,:2] - s[0,:2]
            vline /= np.linalg.norm(vline)

            # loss (to find parallel line)
            l = 1.0 - abs(vline.dot(vtheta))

            # if the line fits
            if l < atol:

                # check for colinear lines
                for line in mw:
                    
                    # distance from pair to line
                    d0 = line[-1,:2] - line[0,:2]
                    d1 = line[0,:2] - s[:,:2]
                    d2 = np.cross(d0, d1)
                    d3 = np.linalg.norm(d0)
                    d = abs(d2)/d3
                    d = np.linalg.norm(d)
                    
                    # if it's colinear, skip
                    skip = True if d < dtol else False
                    if skip:
                        break

                # if we want to skip
                if skip:
                    continue

                # distance from points to line
                d0 = s[-1,:2] - s[0,:2]
                d1 = s[0,:2] - points[:,:2]
                d2 = np.cross(d0, d1)
                d3 = np.linalg.norm(d0)
                d = abs(d2)/d3
                
                # points close to the line
                p = points[d < dtol]

                # sort points along heading line
                d = p - centroid
                d = np.array([np.dot(_[:2], vtheta) for _ in d])
                p = p[np.argsort(d)]

                # record the wall
                mw.append(p)

            # if we found them all
            if len(mw) == n_lines:
                break

        # if we found lines
        if len(mw) > 0:

            # centroid of each line
            d = np.array([_.mean(axis=0) for _ in mw])
            
            # vectors from centroid to line centroids
            d = d - centroid
            
            # cross product of those vector with heading direction
            d = [np.cross(_[:2], vtheta) for _ in d]

            # sort the lines
            i = np.argsort(d)
            mw = [mw[j] for j in np.argsort(d)]

        # return list of wall arrays rather than an array itself
        # because some lines may have different numbers of points
        return mw

    @staticmethod
    def generate_data(points, n_samples, sigma):

        # samples
        samples = np.random.choice(range(points.shape[0]), n_samples)
        samples = points[samples]
        
        # add noise and return
        samples += np.random.normal(
            scale=np.sqrt(sigma), 
            size=(n_samples, 3)
        )
        return samples

    def plot(
        lines=None,
        samples=None,
        means=None,
        sample_lines=None,
        ref_point=None,
        path=None,
        c=None,
        ax=None):

        # make figure and axis
        if ax is None:
            fig, ax = plt.subplots(1)

        # plot lines
        if lines is not None:
            for i, line in enumerate(lines):
                label = 'Ground truth' if i == 0 else None
                ax.plot(
                    line[:,0], line[:,1], 
                    'k.--', label=label, alpha=0.25
                )

        # plot samples
        if samples is not None:
            ax.scatter(
                samples[:,0], samples[:,1], alpha=0.2, 
                label='Samples', c=c, s=40, cmap='viridis'
            )

        # plot means
        if means is not None:
            ax.plot(means[:,0], means[:,1], 'kx', label='Means')

        # plot sample lines
        if sample_lines is not None:
            for i, line in enumerate(sample_lines):

                # port line
                if i == 0:
                    k = 'r-'
                    label = 'Port line'

                # starboard line
                elif i == len(sample_lines) - 1:
                    k = 'g-'
                    label = 'Starboard line'

                # intermediate line
                else:
                    k = 'k-'
                    label = None

                # plot
                ax.plot(
                    line[:,0], line[:,1], 
                    k, alpha=1, label=label
                )
                label = 'Infimum' if i == 0 else None
                ax.plot(
                    line[0,0], line[0,1],
                    'k*', alpha=1, label=label
                )

        # plot reference points
        if ref_point is not None:
            ax.plot(ref_point[0], ref_point[1], 'ko', label='AUV')

        # plot path
        if path is not None:
            ax.plot(path[:,0], path[:,1], 'k.--', label='Path', alpha=0.5)

        # formatting and return
        ax.set_aspect('equal')
        ax.set_xlabel('$x~[m]$')
        ax.set_ylabel('$y~[m]$')
        ax.legend(bbox_to_anchor=(1.05, 1))
        return fig, ax

    @staticmethod
    def approximate(samples, n_components, use_trace=False, model=None):

        '''
        Fits a Gaussian mixture model to cluster
        a set of points.

        Args:
            - samples: (n_samples, n_features) np.array.
            The points you want to cluster.
            - n_components: int.
            The maximum number of clusters.
            - use_trace: bool.
            To use the trace of the covariance matrix
            (total variance) or the determinant 
            (generalized variance).

        Returns:
            - means of active classes
            - covariance matricies of active classes
            - generalized variance of active classes
            - class labels of samples
        '''

        # Variational Gaussian mixture model
        # NOTE: warm-starating isn't good here
        # because it will overfit with too little
        # data points.
        if model is None:
            model = BayesianGaussianMixture(
                n_components=n_components,
                warm_start=True
            )
        else:
            assert model.n_components == n_components
            assert model.warm_start == True

        # optimise the model
        model.fit(samples)

        # compute sample labels
        labels = model.predict(samples)

        # active classes
        i = np.unique(labels)

        # means, covariance matricies
        mu = model.means_[i]
        sigma = model.covariances_[i]

        # total variance
        if use_trace:
            gsigma = np.trace(sigma, axis1=1, axis2=2)

        # generalized variance
        else:
            gsigma = np.linalg.det(sigma)

        # return results
        return mu, sigma, gsigma, labels, model



class A_SetBuoyLocalisationPlan(pt.behaviour.Behaviour):

    def __init__(self):

        # become behaviour
        pt.behaviour.Behaviour.__init__(
            self,
            name=__class__
        )

        # blackboard
        self.bb = pt.blackboard.Blackboard()

    @staticmethod
    def perimeter_plan(point, origin, angle, distances, direction=None):

        # lines at first distance
        lines = [A_SetBuoyLocalisationPlan.perimeter_line(
            centroid=origin,
            angle=angle,
            distance=distances[0],
            face=i,
            side=0
        ) for i in range(4)]
        lines = np.array(lines)

        # distance from point to line centroids
        d = np.mean(lines, axis=1)
        d -= point
        d = np.linalg.norm(d, axis=1)
        
        # closest line and its index
        i = np.argmin(d)
        line = lines[i]

        # if CCW 0 (port) or CW 1 (starboard)
        if direction in [0, 1]:
            side = direction
            line = np.flip(line, axis=0) if direction else line

        # if direction is a vector
        elif np.array(direction).shape == (2,):

            # make unit vector
            direction /= np.linalg.norm(direction)

            # direction of nominal line
            vline = line[1] - line[0]
            vline /= np.linalg.norm(vline)

            # are they in the same direction
            if vline.dot(direction) > 0:

                # continue to the next CCW line
                side = 0
                i = (i+1)%4
                line = lines[i]

            # if they have opposing directions
            else:

                # continue to the next CW line
                side = 1
                i = (i-1)%4
                line = np.flip(lines[i], axis=0)

        # otherwise, just start with the closest point
        else:

            # get closest point in line
            d = line - point
            d = np.linalg.norm(axis=1)

            # ordered line
            side = np.argmin(d)
            line = np.flip(line, axis=0) if side else line

        # intialise path and distance
        path = [line[0]]

        # cycle steps
        j = 0

        # distance counter
        k = 0
        nk = len(distances)

        # make spiral path
        while True:

            # next face
            i = (i-1)%4 if side else (i+1)%4

            # compute next line
            line = A_SetBuoyLocalisationPlan.perimeter_line(
                centroid=origin,
                angle=angle,
                distance=distances[k],
                face=i,
                side=side
            )

            # add first point to path
            path.append(line[0])

            # increment cycle step
            j = (j+1)%4

            # increment distance
            if j == 0:
                k += 1
                
            # termination
            if k == nk:

                # add second point
                if nk > 1:
                    path.append(line[1])

                # terminate
                break

        # return path
        return np.array(path)










        
        

    @staticmethod
    def perimeter_line(centroid, angle, distance, face, side):

        # sanity
        faces0 = [0, 1, 2, 3]
        faces1 = ['right', 'top', 'left', 'bottom']
        assert face in faces0 or face in faces1
        sides0 = [0, 1]
        sides1 = ['port', 'starboard']
        assert side in sides0 or side in sides1
        centroid = centroid[:2]

        # translate
        # this was an afterthough so we can
        # loop through ranges
        if face in faces0:
            face = faces1[face]
        if side in sides0:
            side = sides1[side]

        # unit vectors
        angle = np.deg2rad(angle)
        vtop = np.array([np.cos(angle), np.sin(angle)])
        vbottom = -vtop
        vleft = np.array([-vtop[1], vtop[0]])
        vright = np.array([vtop[1], -vtop[0]])

        # cases
        if face == 'top' and side == 'starboard':
            v0, v1, v2 = vtop, vleft, vright
        elif face == 'top' and side == 'port':
            v0, v1, v2 = vtop, vright, vleft

        elif face == 'bottom' and side == 'starboard':
            v0, v1, v2 = vbottom, vright, vleft
        elif face == 'bottom' and side == 'port':
            v0, v1, v2 = vbottom, vleft, vright

        elif face == 'left' and side == 'starboard':
            v0, v1, v2 = vleft, vbottom, vtop
        elif face == 'left' and side == 'port':
            v0, v1, v2 = vleft, vtop, vbottom

        elif face == 'right' and side == 'starboard':
            v0, v1, v2 = vright, vtop, vbottom
        elif face == 'right' and side == 'port':
            v0, v1, v2 = vright, vbottom, vtop

        # make line
        x = np.array([
            centroid + v0*distance + v1*distance,
            centroid + v0*distance + v2*distance
        ])
        return x


        




class A_SetWallPlan(pt.behaviour.Behaviour):

    def __init__(
        self, 
        name, 
        row_sep,
        depth,
        buoy_link,
        utm_link,
        velocity,
        latlon_utm_serv,
        latlon_utm_serv_alt,
        x0_overshoot,
        x1_overshoot,
        x0_lineup=None,
        x1_lineup=None,
        first_lineup=None,
        starboard=False):

        # become BT
        pt.behaviour.Behaviour.__init__(self, name=name)

        # distance between rows and survey depth
        self.row_sep = row_sep
        self.depth = depth

        # overshoot distance before and after row ends
        self.x0_overshoot = x0_overshoot
        self.x1_overshoot = x1_overshoot

        # distance before and after overshoot for lineup
        self.x0_lineup = x0_lineup
        self.x1_lineup = x1_lineup

        # first line up to ensure good clearance
        self.first_lineup = first_lineup

        # which side of AUV for wall to be on
        self.starboard = starboard

        # TF frames
        self.buoy_link = buoy_link
        self.utm_link = utm_link
        self.tf_listener = tf.TransformListener()

        # need for waypoint class
        self.velocity = velocity
        self.latlon_utm_serv = latlon_utm_serv
        self.latlon_utm_serv_alt = latlon_utm_serv_alt

        # blackboard
        self.bb = pt.blackboard.Blackboard()

        # internal counter
        self.i = 0

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
        return True

    def update(self):

        # if there are no buoys
        if self.bb.get(bb_enums.BUOYS) is None:
            self.bb.set(bb_enums.WALL_PLAN_SET, True)
            # yes, this could be succes,
            # but running forces you to use a condition
            # elsewhere in the BT
            self.feedback_message = 'No buoys'
            return pt.Status.RUNNING

        # print(self.bb.get(bb_enums.BUOYS) )

        try:

            # buoys sorted into walls in map frame
            buoys = self.bb.get(bb_enums.BUOYS)['all']

            # current position in local frame
            point = self.bb.get(bb_enums.LOCATION_POINT_STAMPED)
            point = self.tf_listener.transformPoint(
                self.buoy_link,
                point
            )
            point = np.array([
                point.point.x,
                point.point.y,
                point.point.z
            ])

            # compute a plan
            if buoys is not None:

                # waypoints in local frame
                wps0 = self.full_plan(
                    point,
                    buoys,
                    row_sep=self.row_sep,
                    x0_overshoot=self.x0_overshoot,
                    x1_overshoot=self.x0_overshoot,
                    x0_lineup=self.x0_lineup,
                    x1_lineup=self.x1_lineup,
                    first_lineup=self.first_lineup,
                    starboard=self.starboard,
                    depth=self.depth
                )
                wps1 = self.full_plan(
                    wps0[-1],
                    buoys,
                    row_sep=self.row_sep,
                    x0_overshoot=self.x0_overshoot,
                    x1_overshoot=self.x0_overshoot,
                    x0_lineup=self.x0_lineup,
                    x1_lineup=self.x1_lineup,
                    first_lineup=self.first_lineup,
                    starboard=self.starboard,
                    depth=self.depth
                )
                wps = np.vstack((wps0, wps1))

                # mission waypoints in UTM frame
                mwps = list()

                # loop through waypoints
                for i, wp in enumerate(wps):

                    # make pose
                    pose = PoseStamped()
                    pose.pose.position.x = wp[0]
                    pose.pose.position.y = wp[1]
                    pose.pose.position.z = 1.0 if i == 0 else wp[2]
                    pose.header = Header(frame_id=self.buoy_link)

                    # transform it from local to UTM frame
                    pose = self.tf_listener.transformPose(
                        self.utm_link,
                        pose
                    )

                    # real waypoint
                    wp = Waypoint(
                        maneuver_id='goto',
                        maneuver_imc_id=imc_enums.MANEUVER_GOTO,
                        maneuver_name='wall_following',
                        x=pose.pose.position.x,
                        y=pose.pose.position.y,
                        z=pose.pose.position.z,
                        z_unit=imc_enums.Z_DEPTH,
                        speed=self.velocity,
                        speed_unit=0,
                        tf_frame=self.utm_link,
                        extra_data=None
                    )

                    # add waypoints
                    mwps.append(wp)

            # plandb message
            pdb = PlanDB()
            pdb.request_id = 42
            pdb.plan_id = "WALLS"

            # construct plan
            mission_plan = MissionPlan(
                plandb_msg=pdb,
                latlontoutm_service_name=self.latlon_utm_serv,
                latlontoutm_service_name_alternative=self.latlon_utm_serv_alt,
                plan_frame=self.utm_link,
                waypoints=mwps
            )

            # set the plan and cross our fingers :o
            self.bb.set(bb_enums.MISSION_PLAN_OBJ, mission_plan)
            self.bb.set(bb_enums.WALL_PLAN_SET, True)
            rospy.loginfo('Wall plan set')
            return pt.Status.RUNNING

        except Exception as e:
            raise e
            rospy.loginfo_throttle(60, e)
            return pt.Status.FAILURE

    @staticmethod
    def plan(
        x0, 
        x1, 
        n_rows, 
        row_sep, 
        x0_overshoot, 
        x1_overshoot,
        x0_lineup=None,
        x1_lineup=None,
        starboard=False,
        depth=0.0):

        # sanity
        if x0_lineup is not None:
            assert(x0_overshoot < x0_lineup)
        if x1_lineup is not None:
            assert(x1_overshoot < x1_lineup)

        # parallel unit vector (x,y)
        par = x1[:2] - x0[:2]
        par /= np.linalg.norm(par)
        
        # perpendicular unit vector (x,y)
        perp = np.array([-par[1], par[0]]) if starboard else np.array([par[1], -par[0]])
            
        # construct a lawnmower
        wps = list()

        # i = iteration number, j = row number
        for i, j in enumerate(reversed(range(n_rows))):

            # i = 0, 2, 4, 6, 8, ...
            if not i%2:

                # intial lineup
                if x0_lineup is not None:
                    wps.append(x0[:2] + (row_sep*(j+1))*perp - x0_lineup*par)

                # adjacent to line endpoint
                wps.append(x0[:2] + (row_sep*(j+1))*perp - x0_overshoot*par)
                wps.append(x1[:2] + (row_sep*(j+1))*perp + x1_overshoot*par)

                # terminal lineup
                if x1_lineup is not None:
                    wps.append(x1[:2] + (row_sep*(j+1))*perp + x1_lineup*par)

            # i = 1, 3, 5, 7, 9, ...
            else:

                # initial lineup
                if x1_lineup is not None:
                    wps.append(x1[:2] + (row_sep*(j+1))*perp + x1_lineup*par)

                # adjacent to line endpoint
                wps.append(x1[:2] + (row_sep*(j+1))*perp + x1_overshoot*par)
                wps.append(x0[:2] + (row_sep*(j+1))*perp - x0_overshoot*par)

                # terminal lineup
                if x0_lineup is not None:
                    wps.append(x0[:2] + (row_sep*(j+1))*perp - x0_lineup*par)

        # add depth and return waypoints
        wps = np.array(wps)
        wps = np.hstack((wps, np.full((wps.shape[0], 1), depth)))
        return wps

    @staticmethod
    def full_plan(
        pose, 
        walls,
        row_sep, 
        x0_overshoot, 
        x1_overshoot,
        x0_lineup=None,
        x1_lineup=None,
        first_lineup=None,
        starboard=False,
        depth=1.0):

        # distance between each buoy and AUV
        norms = np.linalg.norm(walls[[0,-1]] - pose, axis=2)

        # index of closest extremal wall
        i = norms.min(axis=1).argmin(axis=0)
        i = 0 if i == 0 else -1
        
        # index of closest extremal buoy 
        norms = np.linalg.norm(walls[i, [0,-1]] - pose, axis=1)
        j = norms.argmin(axis=0)
        j = 0 if j == 0 else -1

        # order the walls based on (i,j)
        walls = np.flip(walls, axis=0) if i == -1 else walls
        walls = np.flip(walls, axis=1) if j == -1 else walls

        # waypoints
        waypoints = list()

        # construct waypoints
        for i, wall in enumerate(walls):

            # i = 0, 2, 4, ...
            if not i%2:
                wps = A_SetWallPlan.plan(
                    x0=wall[0], 
                    x1=wall[-1], 
                    n_rows=1, 
                    row_sep=row_sep, 
                    x0_overshoot=x0_overshoot, 
                    x1_overshoot=x1_overshoot,
                    x0_lineup=first_lineup if i==0 else x0_lineup,
                    x1_lineup=x1_lineup,
                    starboard=starboard,
                    depth=depth
                )
            
            # i = 1, 3, 5, ...
            else:
                wps = A_SetWallPlan.plan(
                    x0=wall[-1], 
                    x1=wall[0], 
                    n_rows=1, 
                    row_sep=row_sep, 
                    x0_overshoot=x0_overshoot, 
                    x1_overshoot=x1_overshoot,
                    x0_lineup=x0_lineup,
                    x1_lineup=x1_lineup,
                    starboard=starboard,
                    depth=depth
                )

            # add waypoint
            waypoints.append(wps)

        # construct path
        waypoints = np.vstack(waypoints)
        # waypoints = np.vstack((pose, waypoints))
        return waypoints
