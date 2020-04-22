#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)
# Mostly a re-write of Christopher's behaviours with more
# descriptive names and atomicaztion of everything

#TODO:
# . A_SetManualWaypoint
# . A_GotoManualWaypoint

# imports courtesy of Chris. TODO cleanup
import py_trees as pt, py_trees_ros as ptr, std_msgs.msg, copy, json, numpy as np
from geodesy.utm import fromLatLong, UTMPoint

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from imc_ros_bridge.msg import PlanControlState
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Bool, Empty
from sam_msgs.msg import PercentStamped


from imc_ros_bridge.msg import EstimatedState, VehicleState


import actionlib_msgs.msg as actionlib_msgs

import rospy
import tf

from sam_globals import *


from bt_common import *

class MissionPlan:
    def __init__(self,
                 actions,
                 utm_zone,
                 utm_band,
                 frame,
                 plan_id=None):
        """
        A container object to keep things related to the mission plan.
        actions is a list of (x,y,z,type:string) tuples
        """
        self.waypoints = []

        # extract the waypoints only
        for a in actions:
            self.waypoints.append(a[:3])

        self.utm_zone = utm_zone
        self.utm_band = utm_band
        self.frame = frame

        if plan_id is None:
            plan_id = "Follow "+str(len(self.waypoints))+" waypoints"
        self.plan_id = plan_id

        self.remaining_wps = self.waypoints
        self.visited_wps = []
        self.current_wp = None
        self.current_wp_index = -1

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


class A_EmergencySurface(pt.behaviour.Behaviour):
    def __init__(self):
        self.bb = pt.blackboard.Blackboard()

        self.emergency_vbs_cmd = None
        self.abort_pub = None
        super(A_EmergencySurface, self).__init__('A_EmergencySurface')

    def setup(self, timeout):
        try:
            self.emergency_vbs_cmd = rospy.Publisher(SAM_VBS_CMD_TOPIC, PercentStamped, queue_size=1)
            self.abort_pub = rospy.Publisher(ABORT_TOPIC, Empty, queue_size=1)
            return True
        except:
            return False


    def update(self):
        self.bb.set(CURRENT_PLAN_ACTION, None)
        self.bb.set(IMC_STATE_BB, IMC_STATE_BLOCKED)

        # latch the abort so the rest of the system can listen to it
        e = Empty()
        self.abort_pub.publish(e)

        # set the VBS to 0, so it empties the tank
        # if someone else is also setting it, nothing else
        # we can do but trust that they will listen to the /abort too
        ps = PercentStamped()
        ps.value = 0
        ps.header.stamp = rospy.Time.now()
        self.emergency_vbs_cmd.publish(ps)

        self.feedback_message = "ABORTED"
        self.bb.set(CURRENTLY_RUNNING_ACTION, 'A_EmergencySurface')
        return pt.Status.RUNNING



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
        plan_str = self.bb.get(MISSION_PLAN_STR_BB)

        # there was no plan to be set
        if plan_str is None or len(plan_str) < MINIMUM_PLAN_STR_LEN:
            self.logger.info("Tried to set bad mission plan:"+str(plan_str))
            return pt.Status.FAILURE

        # there is a plan we can at least look at
        wps_types, zone, band, frame = self.clean(plan_str)
        mission_plan = MissionPlan(actions=wps_types,
                                   utm_zone = zone,
                                   utm_band = band,
                                   frame = frame)

        self.bb.set(MISSION_PLAN_OBJ_BB, mission_plan)
        self.logger.info("Set the mission plan to:"+str(mission_plan.waypoints))

        return pt.Status.SUCCESS

    @staticmethod
    def clean(f):
        """
        Given a json printed string from imc_ros_bridge, cleans it up
        parses it and returns a list of utm xyz waypoints and the utm zone

        By: Christopher Iliffe Sprague (sprague@kth.se)

        """
        #TODO replace with a proper ros message.

        # make the neptus message into a string
        f = str(f)

        # clean the neptus message
        f = f.replace(' ', '')
        f = f.replace('\\n', '')
        f = f.replace('\\"', '"')
        f = f.replace('"\\', '"')
        f = f.replace('\\', '')
        f = f.split(',"transitions":')[0]
        f = f.split('"maneuvers":')[1]
        f = f.replace('\n', '')
        f = f.split(',"transitions"')[0]

        # convert to json
        f = json.loads(f)

        #  json.dump(f, open('/home/ozer/cleaned_up_plan.json', 'w'))

        # convert lat lon to utm
        depths = [float(d['data']['z']) for d in f]

        # ensure signs of depths
        depths = [-d if d > 0 else d for d in depths]

        # get waypoint types
        wtypes = [str(d['data']['abbrev']) for d in f]

        # get latitute and longitude
        f = [fromLatLong(np.degrees(float(d['data']['lat'])), np.degrees(float(d['data']['lon']))) for d in f]

        # get the grid-zone
        gz, band = f[0].gridZone()

        # convert utm to point
        f = [d.toPoint() for d in f]

        # convert point to xyz
        f = [(d.x, d.y, depth, wt) for d, depth, wt in zip(f, depths, wtypes)]

        # return list of utm xyz waypoints and the utm zone
        #TODO unhack this
        # Neptus gives us the plan in UTM coordinates
        frame = '/world_utm'
        return f, gz, band, frame


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

        self.bb = pt.blackboard.Blackboard()

        self.action_goal_handle = None

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name="A_ExecutePlanAction",
            action_spec=MoveBaseAction,
            action_goal=None,
            action_namespace = ACTION_NAMESPACE,
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
            self.listener.waitForTransform("world_utm", BASE_LINK, rospy.Time(), rospy.Duration(4.0))
            return True
        except:
            self.logger.error("Could not find TF!!")
            return False


    def update(self):
        try:
            now = rospy.Time(0)
            (world_trans, world_rot) = self.listener.lookupTransform("world_utm",
                                                                     BASE_LINK,
                                                                     now)
        except (tf.LookupException, tf.ConnectivityException):
            self.logger.warning("Could not get transform between world_utm and "+str(BASE_LINK))
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
        self.estimated_state_pub = rospy.Publisher(ESTIMATED_STATE_TOPIC, EstimatedState, queue_size=1)
        self.plan_control_state_pub = rospy.Publisher(PLAN_CONTROL_STATE_TOPIC, PlanControlState, queue_size=1)
        self.vehicle_state_pub = rospy.Publisher(VEHICLE_STATE_TOPIC, VehicleState, queue_size=1)

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
