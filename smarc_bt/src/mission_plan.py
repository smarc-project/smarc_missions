#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import rospy
import tf
import time
import math
import numpy as np

import common_globals
import imc_enums
import bb_enums

from geometry_msgs.msg import PointStamped, Pose, PoseArray
from geographic_msgs.msg import GeoPoint
from smarc_msgs.srv import LatLonToUTM


class Waypoint:
    def __init__(self,
                 maneuver_id,
                 maneuver_imc_id,
                 maneuver_name,
                 x,
                 y,
                 z,
                 z_unit,
                 speed,
                 speed_unit,
                 tf_frame,
                 extra_data):


        self.maneuver_id = maneuver_id
        self.maneuver_imc_id = maneuver_imc_id
        self.maneuver_name = maneuver_name
        self.x = x
        self.y = y
        self.z = z
        self.z_unit = z_unit
        self.speed = speed
        self.speed_unit = speed_unit
        self.tf_frame = tf_frame
        self.extra_data = extra_data

    def __str__(self):
        s = '{}:{},{},{}'.format(self.maneuver_name,self.x, self.y, self.z)
        return s



class MissionPlan:
    def __init__(self,
                 plandb_msg,
                 latlontoutm_service_name,
                 latlontoutm_service_name_alternative,
                 plan_frame = 'utm',
                 waypoints=None
                 ):
        """
        A container object to keep things related to the mission plan.
        """
        self.plandb_msg = plandb_msg
        self.plan_id = plandb_msg.plan_id
        self.plan_frame = plan_frame

        # test if the service is usable!
        # if not, test the backup
        # if that fails too, raise exception
        self.no_service = False
        self.latlontoutm_service_name = latlontoutm_service_name
        try:
            rospy.loginfo("Waiting (0.5s) lat_lon_to_utm service:{}".format(self.latlontoutm_service_name))
            rospy.wait_for_service(self.latlontoutm_service_name, timeout=0.5)
        except:
            rospy.logwarn(str(self.latlontoutm_service_name)+" service could be connected to!")
            self.latlontoutm_service_name = latlontoutm_service_name_alternative
            rospy.logwarn("Setting the service to the alternative:{}".format(self.latlontoutm_service_name))
            try:
                rospy.loginfo("Waiting (10s) lat_lon_to_utm service alternative:{}".format(self.latlontoutm_service_name))
                rospy.wait_for_service(self.latlontoutm_service_name, timeout=10)
            except:
                rospy.logerr("No lat_lon_to_utm service could be reached! The BT can not accept missions in this state!")
                rospy.logerr("The BT received a mission, tried to convert it to UTM coordinates using {} service and then {} as the backup and neither of them could be reached! Check the navigation/DR stack, the TF tree and the services!".format(latlontoutm_service_name, latlontoutm_service_name_alternative))
                self.no_service = True

        self.aborted = False

        # a list of names for each maneuver
        # good for feedback
        self.waypoint_man_ids = []

        # if waypoints are given directly, then skip reading the plandb message
        if waypoints is None:
            self.waypoints = self.read_plandb(plandb_msg)
        else:
            self.waypoints = waypoints

        for wp in self.waypoints:
            self.waypoint_man_ids.append(wp.maneuver_id)

        # keep track of which waypoint we are going to
        # start at -1 to indicate that _we are not going to any yet_
        self.current_wp_index = -1

        # used to report when the mission was received
        self.creation_time = time.time()


    def latlon_to_utm(self,
                      lat,
                      lon,
                      z):
        rospy.loginfo("Waiting at most 1s for latlontoutm service "+str(self.latlontoutm_service_name))
        try:
            rospy.wait_for_service(self.latlontoutm_service_name, timeout=1)
        except:
            rospy.logwarn(str(self.latlontoutm_service_name)+" service could be connected to! No mission received!")
            return (None, None)

        rospy.loginfo("Got latlontoutm service")
        try:
            latlontoutm_service = rospy.ServiceProxy(self.latlontoutm_service_name,
                                                     LatLonToUTM)
            gp = GeoPoint()
            gp.latitude = np.degrees(lat)
            gp.longitude = np.degrees(lon)
            gp.altitude = z
            res = latlontoutm_service(gp)
            return (res.utm_point.x, res.utm_point.y)
        except rospy.service.ServiceException:
            rospy.logerr_throttle_identical(5, "LatLon to UTM service failed! namespace:{}".format(self.latlontoutm_service_name))
            return (None, None)



    def read_plandb(self, plandb):
        """
        planddb message is a bunch of nested objects,
        we want a list of waypoints in the local frame,
        """
        waypoints = []
        request_id = plandb.request_id
        plan_id = plandb.plan_id
        plan_spec = plandb.plan_spec

        if len(plan_spec.maneuvers) <= 0:
            rospy.logwarn("THERE WERE NO MANEUVERS IN THE PLAN! plan_id:{} (Does this vehicle know of your plan's maneuvers?)".format(plan_id))

        for plan_man in plan_spec.maneuvers:
            man_id = plan_man.maneuver_id
            man_name = plan_man.maneuver.maneuver_name
            man_imc_id = plan_man.maneuver.maneuver_imc_id
            maneuver = plan_man.maneuver
            # probably every maneuver has lat lon z in them, but just in case...
            # goto and sample are identical, with sample having extra "syringe" booleans...
            if man_imc_id == imc_enums.MANEUVER_GOTO or man_imc_id == imc_enums.MANEUVER_SAMPLE:
                if self.no_service:
                    rospy.logwarn("The BT can not reach the latlon_to_utm service! Can not do waypoints!")
                    continue

                utm_x, utm_y = self.latlon_to_utm(maneuver.lat,
                                                  maneuver.lon,
                                                  -maneuver.z)
                if utm_x is None:
                    rospy.loginfo("Could not convert LATLON to UTM! Skipping point:{}".format((maneuver.lat, maneuver.lon, man_name)))
                    continue

                extra_data = {}
                if man_imc_id == imc_enums.MANEUVER_SAMPLE:
                    extra_data = {'syringe0':maneuver.syringe0,
                                  'syringe1':maneuver.syringe1,
                                  'syringe2':maneuver.syringe2}

                # these are in IMC enums, map to whatever enums the action that will consume
                # will need when you are publishing it
                waypoint = Waypoint(
                    maneuver_id = man_id,
                    maneuver_imc_id = man_imc_id,
                    maneuver_name= man_name,
                    tf_frame = 'utm',
                    x = utm_x,
                    y = utm_y,
                    z = maneuver.z,
                    speed = maneuver.speed,
                    z_unit = maneuver.z_units,
                    speed_unit = maneuver.speed_units,
                    extra_data = extra_data
                )
                waypoints.append(waypoint)

            else:
                rospy.logwarn("SKIPPING UNIMPLEMENTED MANEUVER: id:{}, name:{}".format(man_imc_id, man_name))

        if len(waypoints) <= 0:
            rospy.logerr("NO MANEUVERS IN MISSION PLAN!")

        return waypoints



    def get_pose_array(self, flip_z=False):
        pa = PoseArray()
        pa.header.frame_id = self.plan_frame

        # add the rest of the waypoints
        for wp in self.waypoints:
            p = Pose()
            p.position.x = wp.x
            p.position.y = wp.y
            if flip_z:
                p.position.z = -wp.z
            else:
                p.position.z = wp.z
            pa.poses.append(p)

        return pa


    def path_to_list(self, path_msg):
        frame = path_msg.header.frame_id
        if frame != '' and frame != self.plan_frame:
            rospy.logerr_throttle_identical(5, "Waypoints are not in "+self.plan_frame+" they are in "+frame+" !")
            return []

        wps = []
        for pose_stamped in path_msg.poses:
            wp = (
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z
            )
            wps.append(wp)
        return wps


    def __str__(self):
        s = self.plan_id+':\n'
        for wp in self.waypoints:
            s += '\t'+str(wp)+'\n'
        return s



    def is_complete(self):
        # check if we are 'done'
        if self.current_wp_index >= len(self.waypoints):
            # we went tru all wps, we're done
            return True

        return False

    def is_in_progress(self):
        if self.current_wp_index < len(self.waypoints) and\
           self.current_wp_index >= 0:
            return True

        return False


    def visit_wp(self):
        """ call this when you finish going to the wp"""
        if self.is_complete():
            return

        self.current_wp_index += 1


    def get_current_wp(self):
        """
        pop a wp from the remaining wps and return it
        """
        if self.is_complete():
            return None

        # we havent started yet, this is the first time ever
        # someone wanted a wp, meaning we _start now_
        if self.current_wp_index == -1:
            self.current_wp_index = 0

        wp = self.waypoints[self.current_wp_index]
        return wp



