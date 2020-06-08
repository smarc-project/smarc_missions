#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import py_trees as pt
import py_trees_ros as ptr

import time
import numpy as np
from geodesy.utm import fromLatLong, UTMPoint

import rospy
import tf
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs.msg as actionlib_msgs
from geometry_msgs.msg import PointStamped, PoseArray
from nav_msgs.msg import Path
from sam_msgs.msg import ThrusterRPMs, PercentStamped
from std_msgs.msg import Float64, Header, Bool, Empty

# path planner service
from trajectories.srv import trajectory
from std_srvs.srv import SetBool

from imc_ros_bridge.msg import EstimatedState, VehicleState, PlanDB, PlanDBInformation, PlanDBState, PlanControlState, PlanControl, PlanSpecification, Maneuver

import bb_enums
import imc_enums
import common_globals

from mission_plan import MissionPlan


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


class A_SetUTMFromGPS(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Read GPS fix and set our utm band and zone from it.
        Warn when there is a change in it.

        Returns RUNNING until a GPS fix is read.
        Returns SUCCESS afterwards.
        """

        self.bb = pt.blackboard.Blackboard()
        super(A_SetUTMFromGPS, self).__init__("A_SetUTMFromGPS")

        self.gps_zone = None
        self.gps_band = None

        # how many seconds to wait before we complain about bad gps.
        # exponential backoff happens to this until max is reached.
        self._spam_period = 1
        self._max_spam_period = 60


    def update(self):
        data = self.bb.get(bb_enums.GPS_FIX)
        if(data is None or data.latitude is None or data.latitude == 0.0 or data.longitude is None or data.latitude == 0.0 or data.status.status == -1):
            rospy.loginfo_throttle_identical(self._spam_period, "GPS lat/lon are 0s or Nones, cant set utm zone/band from these >:( ")
            # shitty gps
            self._spam_period = min(self._spam_period*2, self._max_spam_period)
            return pt.Status.FAILURE

        self.gps_zone, self.gps_band = fromLatLong(data.latitude, data.longitude).gridZone()

        if self.gps_zone is None or self.gps_band is None:
            return pt.Status.RUNNING

        # first read the UTMs given by ros params
        prev_band = self.bb.get(bb_enums.UTM_BAND)
        prev_zone = self.bb.get(bb_enums.UTM_ZONE)

        if prev_zone != self.gps_zone or prev_band != self.gps_band:
            rospy.logwarn_once("PREVIOUS UTM AND GPS_FIX UTM ARE DIFFERENT!\n Prev:"+str((prev_zone, prev_band))+" gps:"+str((self.gps_zone, self.gps_band)))

            if common_globals.TRUST_GPS:
                rospy.logwarn_once("USING GPS UTM!")
                self.bb.set(bb_enums.UTM_ZONE, self.gps_zone)
                self.bb.set(bb_enums.UTM_BAND, self.gps_band)
            else:
                rospy.logwarn_once("USING PREVIOUS UTM!")
                self.bb.set(bb_enums.UTM_ZONE, prev_zone)
                self.bb.set(bb_enums.UTM_BAND, prev_band)

        return pt.Status.SUCCESS


class A_EmergencySurfaceByForce(pt.behaviour.Behaviour):
    def __init__(self,
                 emergency_topic,
                 vbs_cmd_topic,
                 rpm_cmd_topic,
                 lcg_pid_enable_topic,
                 vbs_pid_enable_topic,
                 tcg_pid_enable_topic,
                 yaw_pid_enable_topic,
                 depth_pid_enable_topic,
                 vel_pid_enable_topic):
        """
        Instead of using an action server, this action
        publishes to the relevant topics directly as a last-resort
        way to do emergency surfacing
        """
        super(A_EmergencySurfaceByForce, self).__init__("A_EmergencySurfaceByForce")

        self.emergency_pub = rospy.Publisher(emergency_topic, Bool, queue_size=10)
        self.vbs_pub = rospy.Publisher(vbs_cmd_topic, PercentStamped, queue_size=10)
        self.rpm_pub = rospy.Publisher(rpm_cmd_topic, ThrusterRPMs, queue_size=10)
        self.lcg_pid_enable = rospy.Publisher(lcg_pid_enable_topic, Bool, queue_size=10)
        self.vbs_pid_enable = rospy.Publisher(vbs_pid_enable_topic, Bool, queue_size=10)
        self.tcg_pid_enable = rospy.Publisher(tcg_pid_enable_topic, Bool, queue_size=10)
        self.yaw_pid_enable = rospy.Publisher(yaw_pid_enable_topic, Bool, queue_size=10)
        self.depth_pid_enable = rospy.Publisher(depth_pid_enable_topic, Bool, queue_size=10)
        self.vel_pid_enable = rospy.Publisher(vel_pid_enable_topic, Bool, queue_size=10)

    def update(self):
        rospy.logerr_throttle(5, "FORCING EMERGENCY SURFACING")
        #Disable controllers
        self.lcg_pid_enable.publish(False)
        self.vbs_pid_enable.publish(False)
        self.tcg_pid_enable.publish(False)
        self.yaw_pid_enable.publish(False)
        self.depth_pid_enable.publish(False)
        self.vel_pid_enable.publish(False)

        #set VBS to 0
        vbs_level = PercentStamped()
        vbs_level.value = 0.0;
        self.vbs_pub.publish(vbs_level)

        # Stop thrusters
        rpm = ThrusterRPMs()
        rpm.thruster_1_rpm = 0.0
        rpm.thruster_2_rpm = 0.0
        self.rpm_pub.publish(rpm)
        return pt.Status.RUNNING



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
            action_spec=MoveBaseAction,
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
        else:
            self.action_server_ok = True

        return True

    def initialise(self):
        if not self.action_server_ok:
            return
        rospy.logwarn("EMERGENCY SURFACING")
        # construct the message
        self.action_goal = MoveBaseGoal()
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


class A_RefineMission(pt.behaviour.Behaviour):
    def __init__(self, path_planner_service_name, path_topic):
        """
        Takes the current mission plan object and run a path planner
        on its waypoints.
        """
        self.bb = pt.blackboard.Blackboard()
        super(A_RefineMission, self).__init__('A_RefineMission')
        self.path_planner_service_name = path_planner_service_name
        self.path_planner = None
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=1)

        self.service_ok = False

    def no_service(self):
        return self.path_planner_service_name is None or self.path_planner_service_name in ['', 'none', 'None', 'null', 'Null', 'NULL']

    def setup(self, timeout):
        if self.no_service():
            return True

        try:
            rospy.wait_for_service(self.path_planner_service_name, timeout)
            self.path_planner = rospy.ServiceProxy(self.path_planner_service_name,
                                                   trajectory)
            self.service_ok = True
        except:
            rospy.logwarn_throttle_identical(5, "Can not reach the path planner at:"+self.path_planner_service_name)

        return True

    def update(self):
        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None or mission_plan.is_complete():
            return pt.Status.FAILURE

        if self.no_service() or not self.service_ok:
            # there is no path planner, just copy the coarse points to the refined side
            mission_plan.set_refined_waypoints(mission_plan.waypoints)
            return pt.Status.SUCCESS

        if len(mission_plan.waypoints) <= 1:
            # there is literally just one point, cant plan for that apparently
            mission_plan.set_refined_waypoints(mission_plan.waypoints)
            return pt.Status.SUCCESS


        # give the location of the auv as the first point in the plan
        # to prevent overshooting the first real waypoint
        ps = self.bb.get(bb_enums.LOCATION_POINT_STAMPED)
        trajectory_response = self.path_planner(mission_plan.get_pose_array(ps))
        refined_path = trajectory_response.fine
        refined_path.header.frame_id = 'map'
        self.path_pub.publish(refined_path)
        mission_plan.set_refined_waypoints(mission_plan.path_to_list(refined_path))
        rospy.loginfo_throttle_identical(10, "Refined waypoints length:"+str(len(mission_plan.refined_waypoints)))
        return pt.Status.SUCCESS




class A_SetNextPlanAction(pt.behaviour.Behaviour):
    def __init__(self, do_not_visit=False):
        """
        Sets the current plan action to the next one
        RUNNING if it can set it to something that is not None
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
            return pt.Status.FAILURE

        rospy.loginfo_throttle_identical(5, "Set CURRENT_PLAN_ACTION to:"+str(next_action))
        self.bb.set(bb_enums.CURRENT_PLAN_ACTION, next_action)
        return pt.Status.SUCCESS



class A_GotoWaypoint(ptr.actions.ActionClient):
    def __init__(self, action_namespace):
        """
        Runs an action server that will move the robot to the given waypoint
        """

        self.bb = pt.blackboard.Blackboard()
        list_of_maneuvers = self.bb.get(bb_enums.MANEUVER_ACTIONS)
        if list_of_maneuvers is None:
            list_of_maneuvers = ["A_GotoWaypoint"]
        else:
            list_of_maneuvers.append("A_GotoWaypoint")
        self.bb.set(bb_enums.MANEUVER_ACTIONS, list_of_maneuvers)
        self.action_goal_handle = None

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name="A_GotoWaypoint",
            action_spec=MoveBaseAction,
            action_goal=None,
            action_namespace = action_namespace,
            override_feedback_message_on_running="Moving to waypoint"
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
        if not self.action_server_ok:
            return

        wp, frame = self.bb.get(bb_enums.CURRENT_PLAN_ACTION)
        # if this is the first ever action, we need to get it ourselves
        if wp is None:
            rospy.logwarn("No wp found to execute! Was A_SetNextPlanAction called before this?")
            return

        # construct the message
        self.action_goal = MoveBaseGoal()
        self.action_goal.target_pose.pose.position.x = wp[0]
        self.action_goal.target_pose.pose.position.y = wp[1]
        self.action_goal.target_pose.pose.position.z = wp[2]
        self.action_goal.target_pose.header.frame_id = frame
        rospy.loginfo("Goto waypoint action goal initialized")

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
        if result:
            self.feedback_message = "Completed goal"
            rospy.loginfo(self.feedback_message)
            return pt.Status.SUCCESS


        return pt.Status.RUNNING

    def feedback_cb(self, msg):
        pass


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
        try:
            (world_trans, world_rot) = self.listener.lookupTransform(self.utm_link,
                                                                     self.base_link,
                                                                     rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logwarn_throttle_identical(5, "Could not get transform between "+ self.utm_link +" and "+ self.base_link)
            return pt.Status.FAILURE
        except:
            rospy.logwarn_throttle_identical(5, "Could not do tf lookup for some other reason")
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

        return pt.Status.SUCCESS


class A_UpdateLatLon(pt.behaviour.Behaviour):
    """
    We use the tf to update our lat/lon instead of the actual GPS topic because
    we might not have GPS, but tf is updated by dead-reckoninig and does all
    kinds of filtering.
    """
    def __init__(self):
        super(A_UpdateLatLon, self).__init__("A_UpdateLatLon")
        self.bb = pt.blackboard.Blackboard()

    def update(self):
        world_trans = self.bb.get(bb_enums.WORLD_TRANS)

        # get the utm zone of our current lat,lon
        utmz = self.bb.get(bb_enums.UTM_ZONE)
        band = self.bb.get(bb_enums.UTM_BAND)

        if utmz is None or band is None or world_trans is None:
            reason = "Could not update current lat/lon!"
            rospy.logwarn_throttle_identical(5, reason)
            self.feedback_message = reason
            return pt.Status.FAILURE


        # get positional feedback of the p2p goal
        easting, northing = world_trans[0], world_trans[1]
        # make utm point
        pnt = UTMPoint(easting=easting, northing=northing, altitude=0, zone=utmz, band=band)
        # get lat-lon
        pnt = pnt.toMsg()
        self.bb.set(bb_enums.CURRENT_LATITUDE, pnt.latitude)
        self.bb.set(bb_enums.CURRENT_LONGITUDE, pnt.longitude)
        return pt.Status.SUCCESS


class A_UpdateNeptusPlanControl(pt.behaviour.Behaviour):
    def __init__(self, plan_control_topic):
        super(A_UpdateNeptusPlanControl, self).__init__("A_UpdateNeptusPlanControl")
        self.bb = pt.blackboard.Blackboard()
        self.sub = rospy.Subscriber(plan_control_topic, PlanControl, self.plancontrol_cb)
        self.plan_control_msg = None

    def plancontrol_cb(self, plan_control_msg):
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
                rospy.loginfo_throttle_identical(20, "Started plan:"+str(plan_id))
            else:
                if current_mission_plan is None:
                    rospy.logwarn_throttle_identical(10, "Start given for plan:"+str(plan_id)+" but we don't have a plan!:")
                else:
                    rospy.logwarn_throttle_identical(10, "Start given for plan:"+str(plan_id)+" our plan:"+str(current_mission_plan.plan_id))

        if typee==0 and op==1 and plan_id=='' and flags==1:
            # stop button
            self.bb.set(bb_enums.PLAN_IS_GO, False)
            self.bb.set(bb_enums.ENABLE_AUTONOMY, False)

        # this string is hardcoded in Neptus, so we hardcode it here too!
        if typee==0 and op==0 and plan_id=='teleoperation-mode' and flags==0:
            # teleop button
            self.bb.set(bb_enums.ENABLE_AUTONOMY, True)
            rospy.logwarn_throttle_identical(10, "AUTONOMOUS MODE")

        return pt.Status.SUCCESS



class A_UpdateNeptusEstimatedState(pt.behaviour.Behaviour):
    def __init__(self, estimated_state_topic):
        super(A_UpdateNeptusEstimatedState, self).__init__("A_UpdateNeptusEstimatedState")
        self.bb = pt.blackboard.Blackboard()
        self.estimated_state_pub = rospy.Publisher(estimated_state_topic, EstimatedState, queue_size=1)

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

        # construct message for neptus
        e_state = EstimatedState()
        e_state.lat = np.radians(lat)
        e_state.lon= np.radians(lon)
        e_state.depth = depth
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(world_rot)
        e_state.psi = np.pi/2. - yaw

        # send the message to neptus
        self.estimated_state_pub.publish(e_state)
        return pt.Status.SUCCESS


class A_UpdateNeptusPlanControlState(pt.behaviour.Behaviour):
    def __init__(self, plan_control_state_topic):
        super(A_UpdateNeptusPlanControlState, self).__init__("A_UpdateNeptusPlanControlState")
        self.bb = pt.blackboard.Blackboard()
        self.plan_control_state_pub = rospy.Publisher(plan_control_state_topic, PlanControlState, queue_size=1)

    def update(self):
        # construct current progress message for neptus
        msg = PlanControlState()
        tip_name = self.bb.get(bb_enums.TREE_TIP_NAME)
        tip_status = self.bb.get(bb_enums.TREE_TIP_STATUS)

        # this tip_status looks like: "Status.FAILURE"
        # I just wanna get the first letter after dot.
        msg.man_id = tip_name+'('+tip_status[7]+')'

        mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None or mission_plan.is_complete():
            msg.plan_id = 'No plan'
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
        self.vehicle_state_pub = rospy.Publisher(vehicle_state_topic, VehicleState, queue_size=1)

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
    def __init__(self, plandb_topic, utm_link, local_link):
        super(A_UpdateNeptusPlanDB, self).__init__("A_UpdateNeptusPlanDB")
        self.bb = pt.blackboard.Blackboard()
        # neptus sends lat/lon, which we convert to utm, which we then convert to local
        self.utm_link = utm_link
        self.local_link = local_link

        # the message body is largely the same, so we can re-use most of it
        self.plandb_msg = PlanDB()
        self.plandb_msg.type = imc_enums.PLANDB_TYPE_SUCCESS
        self.plandb_msg.op = imc_enums.PLANDB_OP_SET

        self.plandb_pub = rospy.Publisher(plandb_topic, PlanDB, queue_size=1)
        self.plandb_sub = rospy.Subscriber(plandb_topic, PlanDB, callback=self.plandb_cb, queue_size=1)
        self.latest_plandb_msg = None


    def plandb_cb(self, plandb_msg):
        """
        as an answer to OUR answer of 'type=succes, op=set', neptus sends a 'type=request, op=get_info'.
        """
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
                                   local_frame = self.local_link,
                                   plandb_msg = plandb_msg)


        self.bb.set(bb_enums.MISSION_PLAN_OBJ, mission_plan)
        self.bb.set(bb_enums.ENABLE_AUTONOMY, False)
        rospy.loginfo_throttle_identical(5, "Set the mission plan to:"+str(mission_plan.waypoints))


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
            rospy.loginfo_throttle_identical(20, "Received SUCCESS for plandb set")

        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_GET_INFO:
            rospy.loginfo_throttle_identical(20, "Received SUCCESS for plandb get info")

        elif typee == imc_enums.PLANDB_TYPE_SUCCESS and op == imc_enums.PLANDB_OP_GET_STATE:
            rospy.loginfo_throttle_identical(20, "Received SUCCESS for plandb get state")

        elif op == imc_enums.PLANDB_OP_SET:
            self.handle_set_plan(plandb_msg)

        else:
            rospy.loginfo_throttle_identical(5, "Received some unhandled planDB message:\n"+str(plandb_msg))




    def respond_set_success(self):
        current_mission_plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_mission_plan is None:
            rospy.logwarn_throttle_identical(30, "No mission plan obj!")
            return

        plan_id = current_mission_plan.plan_id
        self.plandb_msg.plan_id = plan_id
        self.plandb_pub.publish(self.plandb_msg)
        rospy.loginfo_throttle_identical(30, "Answered set success for plan_id:"+str(plan_id))

    def update(self):
        # we just want to tell neptus we got the plan all the time
        # this keeps the thingy green
        self.respond_set_success()
        self.handle_plandb_msg()
        return pt.Status.SUCCESS


class A_UpdateMissonForPOI(pt.behaviour.Behaviour):
    """
    creates a new diamond-shaped mission over a detected POI
    and sets that as the current mission plan.
    always returns SUCCESS
    """
    def __init__(self, utm_link, local_link, poi_link):
        super(A_UpdateMissonForPOI, self).__init__(name="A_UpdateMissonForPOI")
        self.bb = pt.blackboard.Blackboard()
        self.utm_link = utm_link
        self.local_link = local_link
        self.poi_link = poi_link
        self.tf_listener = tf.TransformListener()

        self.poi_link_available = False


    def setup(self, timeout):
        try:
            rospy.loginfo_throttle(3, "Waiting for transform from {} to {}...".format(self.poi_link, self.local_link))
            self.tf_listener.waitForTransform(self.poi_link, self.local_link, rospy.Time(), rospy.Duration(timeout))
            rospy.loginfo_throttle(3, "...Got it")
            self.poi_link_available = True
        except:
            rospy.logerr_throttle(5, "Could not find tf from:"+self.poi_link+" to:"+self.local_link+" disabling updates")

        return True

    def update(self):
        if not self.poi_link_available:
            return pt.Status.FAILURE

        poi = self.bb.get(bb_enums.POI_POINT_STAMPED)
        if poi is None:
            return pt.Status.SUCCESS
        poi_local = self.tf_listener.transformPoint(self.local_link, poi)

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
                                   local_frame = self.local_link,
                                   plandb_msg = pdb,
                                   waypoints = waypoints,
                                   waypoint_man_ids=waypoint_man_ids)

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
        self.pa_pub = rospy.Publisher(plan_viz_topic, PoseArray, queue_size=1)

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
            action_spec=MoveBaseAction,
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
        self.action_goal = MoveBaseGoal()
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
