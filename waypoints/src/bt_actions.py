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
from std_msgs.msg import Float64

import actionlib_msgs.msg as actionlib_msgs

import rospy
import tf


from bt_common import *

class MissionPlan:
    def __init__(self,
                 actions,
                 utm_zone,
                 utm_band):
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

        self.remaining_wps = self.waypoints
        self.visited_wps = []
        self.current_wp = None

    def pop_wp(self):
        """
        pop a wp from the remaining wps and return it
        """
        if len(self.remaining_wps) > 0:
            self.current_wp = self.remaining_wps[0]
            self.remaining_wps = self.remaining_wps[1:]
        else:
            self.current_wp = None

        return self.current_wp

    def visit(self):
        self.visited_wps.append(self.current_wp)
        self.current_wp = None


class A_GetGPSFix(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Listens to a gps fix, puts it in the blackboard and sets the utm zone and band
        stuff once.

        Returns FAILURE if there is nothing in the gps topic AND there was no GPS fixes before
        """

        self.bb = pt.blackboard.Blackboard()

        self.gps_sub = rospy.Subscriber(name=SAM_GPS_TOPIC, data_class=NavSatFix, callback=self.gps_cb)
        self.msg = None
        self.set_utm_stuff = False

        super(A_GetGPSFix, self).__init__(name="A_GetGPSFix")

    def gps_cb(self, data):
        self.msg = data

    def update(self):
        if self.msg is not None:
            msg = self.msg
            self.bb.set(GPS_FIX_BB, msg)

            # we wanna do this calculation just once
            if not self.set_utm_stuff:
                f = fromLatLong(msg.latitude, msg.longitude)
                gz, band = f.gridZone()
                self.bb.set(UTM_BAND_BB, band)
                self.bb.set(UTM_ZONE_BB, gz)
                self.set_utm_stuff = True

            return pt.Status.SUCCESS

        elif self.msg is None and self.bb.get(GPS_FIX_BB) is not None:
            return pt.Status.SUCCESS
        else:
            return pt.Status.FAILURE




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
        plan_str = self.bb.get(MISSION_PLAN_STR)

        # there was no plan to be set
        if plan_str is None or len(plan_str) < MINIMUM_PLAN_STR_LEN:
            rospyl.loginfo("Mission plan (set) was bad:"+str(plan_str))
            return pt.Status.FAILURE

        # there is a plan we can at least look at
        wps_types, zone, band = self.clean(plan_str)
        mission_plan = MissionPlan(actions=wps_types,
                                   utm_zone = zone,
                                   utm_band = band)

        self.bb.set(MISSION_PLAN_OBJ, mission_plan)
        rospy.loginfo("Set the mission plan to:"+str(mission_plan.waypoints))
        self.bb.set(CURRENT_PLAN_ACTION, mission_plan.pop_wp())
        return pt.Status.SUCCESS

    @staticmethod
    def clean(f):
        """
        Given a json printed string from imc_ros_bridge, cleans it up
        parses it and returns a list of utm xyz waypoints and the utm zone

        By: Christopher Iliffe Sprague (sprague@kth.se)
        """

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

        #json.dump(f, open('plan.json', 'w'))

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
        return f, gz, band


class A_SetNextPlanAction(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Sets the current plan action to the next one
        SUCCESS if it can set it to something that is not None
        FAILURE otherwise
        """
        self.bb = pt.blackboard.Blackboard()
        super(A_SetNextPlanAction, self).__init__('A_SetNextPlanAction')

    def update(self):
        mission_plan = self.bb.get(MISSION_PLAN_OBJ)
        next_action = mission_plan.pop_wp()
        rospy.loginfo("Set CURRENT_PLAN_ACTION to"+str(next_action))
        self.bb.set(CURRENT_PLAN_ACTION, next_action)

        if next_action is None:
            return pt.Status.FAILURE

        return pt.Status.SUCCESS



class A_ExecutePlanAction(ptr.actions.ActionClient):
    def __init__(self):
        """
        Executes the currently set plan action in the blackboard

        Copied from Chris's code mostly
        """
        #TODO support more than waypoints

        self.bb = pt.blackboard.Blackboard()

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name="A_ExecutePlanAction",
            action_spec=MoveBaseAction,
            action_goal=None,
            action_namespace="/bezier_planner",
            override_feedback_message_on_running="Moving to waypoint"
        )


    def initialise(self):
        wp = self.bb.get(CURRENT_PLAN_ACTION)

        # construct the message
        self.action_goal = MoveBaseGoal()
        self.action_goal.target_pose.pose.position.x = wp[0]
        self.action_goal.target_pose.pose.position.y = wp[1]
        self.action_goal.target_pose.pose.position.z = wp[2]

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
            return pt.Status.INVALID

        # if goal hasn't been sent yet
        if not self.sent_goal:
            self.action_client.send_goal(self.action_goal, feedback_cb=self.feedback_cb)
            self.sent_goal = True
            rospy.loginfo("Sent goal to bezier planner:"+str(self.action_goal))
            return pt.Status.RUNNING


        # if the goal was aborted or preempted
        if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                              actionlib_msgs.GoalStatus.PREEMPTED]:
            return pt.Status.FAILURE

        result = self.action_client.get_result()

        # if the goal was accomplished
        if result:
            return pt.Status.SUCCESS

        # if we're still trying to accomplish the goal
        else:
            return pt.Status.RUNNING

    def feedback_cb(self, msg):
        self.bb.set(LAST_PLAN_ACTION_FEEDBACK, msg)


#TODO split the data gathering part from this
# and make that its own subtree before everything
class A_PublishToNeptus(pt.behaviour.Behaviour):
    def __init__(self):
        """
        Publish some feedback to Neptus.
        Always returns SUCCESS
        """
        self.bb = pt.blackboard.Blackboard()
        self.estimated_state_pub = rospy.Publisher(ESTIMATED_STATE_TOPIC, Pose, queue_size=1)
        self.plan_control_state_pub = rospy.Publisher(PLAN_CONTROL_STATE_TOPIC, PlanControlState, queue_size=1)


        self.listener = tf.TransformListener()
        self.listener.waitForTransform("world_utm", BASE_LINK, rospy.Time(), rospy.Duration(4.0))

        self.depth_sub = rospy.Subscriber(DEPTH_TOPIC, Float64, self.depth_cb)
        self.depth = -9999

        super(A_PublishToNeptus, self).__init__("A_PublishToNeptus")

    def update_estimated_state(self):
        try:
            now = rospy.Time(0)
            (world_trans, world_rot) = self.listener.lookupTransform("world_utm",
                                                                     BASE_LINK,
                                                                     now)
        except (tf.LookupException, tf.ConnectivityException):
            print("Could not get transform between world_utm and",BASE_LINK)

        # get positional feedback of the p2p goal
        orientation = Quaternion(*world_rot)
        msg = world_trans

        # get the utm zone of our current lat,lon
        utmz = self.bb.get(UTM_ZONE_BB)
        band = self.bb.get(UTM_BAND_BB)

        if utmz is None or band is None:
            rospy.loginfo("Utmz or band was None!!")
            return pt.Status.FAILURE

        # make utm point
        pnt = UTMPoint(easting=msg[0], northing=msg[1], altitude=0, zone=utmz, band=band)
        # get lat-lon
        pnt = pnt.toMsg()
        # construct message for neptus
        mmsg = Pose()
        mmsg.position.x = np.radians(pnt.longitude)
        mmsg.position.y = np.radians(pnt.latitude)
        mmsg.position.z = self.depth
        mmsg.orientation = orientation
        # send the message to neptus
        self.estimated_state_pub.publish(mmsg)

    def update_plan_control_state(self):
        # construct current progress message for neptus
        msg = PlanControlState()
        mission_plan = self.bb.get(MISSION_PLAN_OBJ)
        if mission_plan is None:
            return pt.Status.FAILURE

        current_wp = mission_plan.current_wp
        num_remaining = len(mission_plan.remaining_wps)
        num_done = len(mission_plan.visited_wps)
        total = len(mission_plan.waypoints)

        msg.man_id = "Going to wp:"+str(current_wp)+" remaining:"+str(num_remaining)+" visited:"+str(num_done)
        msg.plan_progress = 100* (num_done / total)

        # send message to neptus
        self.plan_control_state_pub.publish(msg)

    def depth_cb(self, data):
        self.depth = data.data

    def update(self):
        """
        mostly scavenged from Chris's stuff (sprague@kth.se)
        """
        self.update_estimated_state()
        self.update_plan_control_state()

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
