#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

from geodesy.utm import fromLatLong
import rospy
import tf
import time
import math
import numpy as np

import common_globals
import imc_enums
import bb_enums

class MissionPlan:
    def __init__(self,
                 plan_frame,
                 local_frame,
                 plandb_msg):
        """
        A container object to keep things related to the mission plan.
        """
        self.plandb_msg = plandb_msg
        self.local_frame = local_frame
        self.plan_id = plandb_msg.plan_id
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform(plan_frame, local_frame, rospy.Time(), rospy.Duration(4.0))
        except:
            rospy.logerr_throttle(5, "Could not find tf from:"+plan_frame+" to:"+local_frame)

        self.waypoints = self.read_plandb(plandb_msg, plan_frame, local_frame)
        self.refined_waypoints = None

        # keep track of which waypoint we are going to
        self.current_wp_index = 0
        self.current_refined_wp_index = 0

        # used to report when the mission was received
        self.creation_time = time.time()


    def read_plandb(self, plandb, plan_frame, local_frame):
        """
        planddb message is a bunch of nested objects,
        we want a list of waypoints in the local frame,
        """
        waypoints = []
        request_id = plandb.request_id
        plan_id = plandb.plan_id
        plan_spec = plandb.plan_spec

        for plan_man in plan_spec.maneuvers:
            man_id = plan_man.maneuver_id
            man_name = plan_man.maneuver.maneuver_name
            man_imc_id = plan_man.maneuver.maneuver_imc_id
            maneuver = plan_man.maneuver
            # probably every maneuver has lat lon z in them, but just in case...
            if man_imc_id == imc_enums.MANEUVER_GOTO:
                lat = maneuver.lat
                lon = maneuver.lon
                depth = maneuver.z
                utm_point = fromLatLong(np.degrees(lat), np.degrees(lon)).toPoint()
                trans, rot = self.tf_listener.lookupTransform(plan_frame,
                                                              local_frame,
                                                              rospy.Time(0))
                waypoint = (utm_point.x + trans[0], utm_point.y + trans[1], depth)
                waypoints.append(waypoint)
            else:
                rospy.logwarn("SKIPPING UNIMPLEMENTED MANEUVER:", man_imc_id, man_name)

        return waypoints

    def __str__(self):
        return 'wps:'+str(self.waypoints)+'\nremaining:'+str(self.remaining_wps)


    def set_refined_waypoints(self, refined_waypoints):
        """
        given the waypoints in the plan, a path planner
        should create a more detailed and kinematically possible path
        to follow, we will keep that in this object too
        """
        self.refined_waypoints = refined_waypoints


    def is_complete(self):
        # check if we are 'done'
        if self.refined_waypoints is None:
            # not even refined, we are def. not done
            return False

        if self.current_refined_wp_index >= len(self.refined_waypoints) or \
           self.current_wp_index >= len(self.waypoints):
            # we went tru all wps, we're done
            return True

        return False


    def visit_wp(self):
        """ call this when you finish going to the wp you received by pop """
        ref_wp = self.refined_waypoints[self.current_refined_wp_index]
        coarse_wp = self.waypoints[self.current_wp_index]
        self.current_refined_wp_index += 1
        # check if the refined waypoint is close to a 'real' waypoint
        # if it is, we can count the 'real' wp as reached too
        diff = math.sqrt((ref_wp[0]-coarse_wp[0])**2 + (ref_wp[1]-coarse_wp[1])**2)
        if diff < common_globals.COARSE_PLAN_REFINED_PLAN_THRESHOLD:
            self.current_wp_index += 1


    def pop_wp(self):
        """
        pop a wp from the remaining wps and return it
        """
        if self.is_complete() or self.refined_waypoints is None:
            return None
        ref_wp = self.refined_waypoints[self.current_refined_wp_index]
        return ref_wp, self.local_frame



