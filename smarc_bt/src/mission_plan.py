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
from smarc_msgs.msg import GotoWaypointGoal, GotoWaypoint

from coverage_planner import create_coverage_path

class Waypoint:
    def __init__(self,
                 goto_waypoint = None,
                 imc_man_id = None,
                 extra_data = None):
        """
        goto_waypoint message reference:
            uint8 Z_CONTROL_NONE=0
            uint8 Z_CONTROL_DEPTH=1
            uint8 Z_CONTROL_ALTITUDE=2
            uint8 SPEED_CONTROL_NONE=0
            uint8 SPEED_CONTROL_RPM=1
            uint8 SPEED_CONTROL_SPEED=2
            geometry_msgs/PoseStamped pose
              std_msgs/Header header
                uint32 seq
                time stamp
                string frame_id
              geometry_msgs/Pose pose
                geometry_msgs/Point position
                  float64 x
                  float64 y
                  float64 z
                geometry_msgs/Quaternion orientation
                  float64 x
                  float64 y
                  float64 z
                  float64 w
            float64 goal_tolerance
            uint8 z_control_mode
            float64 travel_altitude
            float64 travel_depth
            uint8 speed_control_mode
            float64 travel_rpm
            float64 travel_speed
            float64 lat
            float64 lon
            string name
        """


        # this is a numerical "type" of maneuver identifier used
        # only for imc/neptus
        self.imc_man_id = imc_man_id
        # the GotoWaypoint object from smarc_msgs.msg
        self.wp = goto_waypoint
        self.extra_data = extra_data


    def set_utm_from_latlon(self, lat_lon_to_utm_serv):
        gp = GeoPoint()
        gp.latitude = self.wp.lat
        gp.longitude = self.wp.lon
        gp.altitude = 0
        res = lat_lon_to_utm_serv(gp)
        self.wp.pose.pose.position.x = res.utm_point.x
        self.wp.pose.pose.position.y = res.utm_point.y

    def set_latlon_from_utm(self, utm_to_lat_lon_serv):
        p = Point()
        p.x = self.x
        p.y = self.y
        p.z = -self.wp.travel_depth
        res = utm_to_lat_lon_serv(p)
        self.wp.lat = res.lat_lon_point.latitude
        self.wp.lon = res.lat_lon_point.longitude

    def is_too_similar_to_other(self, other_wp):
        """
        other_wp is a smarc_msgs GotoWaypoint
        """
        xy_tolerance = min(self.wp.goal_tolerance, other_wp.goal_tolerance)
        z_tolerance = 0.6 # hardcoded... i dont know what harsha is doing here?
        rpm_tolerance = 50 #rpm
        speed_tolerance = 0.1 #m/s

        xy_too_close = False
        z_too_close = False
        speed_too_close = False

        # if the control mode is different, they _are_ different wps
        if self.wp.z_control_mode != other_wp.z_control_mode:
            return False
        # otherwise, we gotta see if the _amounts_ are different enough
        else:
            if self.wp.z_control_mode == GotoWaypoint.Z_CONTROL_DEPTH:
                z_too_close = abs(self.wp.travel_depth - other_wp.travel_depth) < z_tolerance

            if self.wp.z_control_mode == GotoWaypoint.Z_CONTROL_ALTITUDE:
                z_too_close = abs(self.wp.travel_altitude - other_wp.travel_altitude) < z_tolerance

        # same with speed
        if self.wp.speed_control_mode != other_wp.speed_control_mode:
            return False
        else:
            if self.wp.speed_control_mode == GotoWaypoint.SPEED_CONTROL_RPM:
                speed_too_close = abs(self.wp.travel_rpm - other_wp.travel_rpm) < rpm_tolerance

            if self.wp.speed_control_mode == GotoWaypoint.SPEED_CONTROL_SPEED:
                speed_too_close = abs(self.wp.travel_speed - other_wp.travel_speed) < speed_tolerance

        xy_dist = math.sqrt( (self.x - other_wp.pose.pose.position.x)**2 + (self.y - other_wp.pose.pose.position.y)**2 )
        xy_too_close = xy_dist < xy_tolerance

        # if all of them are too close, wps are similar
        # otherwise they are different enough
        # if the control modes are different, that was handled above already
        return all((xy_too_close, z_too_close, speed_too_close))





    @property
    def x(self):
        return self.wp.pose.pose.position.x

    @property
    def y(self):
        return self.wp.pose.pose.position.y

    @property
    def depth(self):
        return self.wp.travel_depth

    @property
    def frame_id(self):
        return self.wp.pose.header.frame_id

    @property
    def is_actionable(self):
        if (self.x == 0 and self.y == 0) and self.frame_id == 'utm':
            return False

        return True

    def read_imc_maneuver(self, maneuver, utm_x, utm_y, extra_data=None):
        gwp  = GotoWaypoint()
        gwp.pose.header.frame_id = 'utm'
        gwp.pose.pose.position.x = utm_x
        gwp.pose.pose.position.y = utm_y
        gwp.lat = maneuver.lat
        gwp.lon = maneuver.lon
        gwp.name = maneuver.maneuver_name
        gwp.goal_tolerance = 2 # to make this reactive, whoever sends the WP should set it

        # convert the IMC enums into SMaRC enums
        if maneuver.speed_units == imc_enums.SPEED_UNIT_RPM:
            gwp.speed_control_mode = GotoWaypoint.SPEED_CONTROL_RPM
            gwp.travel_rpm = maneuver.speed
        elif maneuver.speed_units == imc_enums.SPEED_UNIT_MPS:
            gwp.speed_control_mode = GotoWaypoint.SPEED_CONTROL_SPEED
            gwp.travel_speed = maneuver.speed
        else:
            gwp.speed_control_mode = GotoWaypointGoal.SPEED_CONTROL_NONE
            rospy.logwarn("Speed control of the waypoint is NONE!")

        gwp.z_control_mode = maneuver.z_units # same in smarc and imc
        if maneuver.z_units == imc_enums.Z_DEPTH:
            gwp.travel_depth = maneuver.z
        elif maneuver.z_units == imc_enums.Z_ALTITUDE:
            gwp.travel_altitude = maneuver.z
        else:
            rospy.logwarn("Z control mode not depth or alt, defaulting to 0 depth!")
            gwp.travel_depth = 0
            gwp.z_control_mode = GotoWaypoint.Z_CONTROL_DEPTH

        self.imc_man_id = maneuver.maneuver_imc_id
        self.wp = gwp
        self.extra_data = extra_data


    def __str__(self):
        s = 'Man: {}'.format(self.wp)
        return s



class MissionPlan:
    def __init__(self,
                 plandb_msg,
                 auv_config,
                 coverage_swath = None,
                 vehicle_localization_error_growth = None,
                 waypoints=None
                 ):
        """
        A container object to keep things related to the mission plan.
        """
        self.plandb_msg = plandb_msg
        if plandb_msg is not None:
            self.plan_id = plandb_msg.plan_id
        else:
            self.plan_id = 'NOPLAN'

        self.plan_frame = auv_config.UTM_LINK
        self.coverage_swath = coverage_swath
        self.vehicle_localization_error_growth = vehicle_localization_error_growth

        # test if the service is usable!
        # if not, test the backup
        # if that fails too, raise exception
        self.no_service = False
        self.latlontoutm_service_name = auv_config.LATLONTOUTM_SERVICE
        try:
            rospy.loginfo("Waiting (0.5s) lat_lon_to_utm service:{}".format(self.latlontoutm_service_name))
            rospy.wait_for_service(self.latlontoutm_service_name, timeout=0.5)
        except:
            rospy.logwarn(str(self.latlontoutm_service_name)+" service could be connected to!")
            self.latlontoutm_service_name = auv_config.LATLONTOUTM_SERVICE_ALTERNATIVE
            rospy.logwarn("Setting the service to the alternative:{}".format(self.latlontoutm_service_name))
            try:
                rospy.loginfo("Waiting (10s) lat_lon_to_utm service alternative:{}".format(self.latlontoutm_service_name))
                rospy.wait_for_service(self.latlontoutm_service_name, timeout=10)
            except:
                rospy.logerr("No lat_lon_to_utm service could be reached! The BT can not accept missions in this state!")
                rospy.logerr("The BT received a mission, tried to convert it to UTM coordinates using {} service and then {} as the backup and neither of them could be reached! Check the navigation/DR stack, the TF tree and the services!".format(latlontoutm_service_name, latlontoutm_service_name_alternative))
                self.no_service = True


        # a list of names for each maneuver
        # good for feedback
        self.waypoint_man_ids = []

        # if waypoints are given directly, then skip reading the plandb message
        if waypoints is None:
            self.waypoints = self.read_plandb(plandb_msg)
        else:
            self.waypoints = waypoints

        for wp in self.waypoints:
            self.waypoint_man_ids.append(wp.wp.name)

        # keep track of which waypoint we are going to
        # start at -1 to indicate that _we are not going to any yet_
        self.current_wp_index = -1

        # used to report when the mission was received
        self.creation_time = time.time()

        # state of this plan
        self.plan_is_go = False

    def latlon_to_utm(self,
                      lat,
                      lon,
                      z,
                      in_degrees=False):
        try:
            rospy.wait_for_service(self.latlontoutm_service_name, timeout=1)
        except:
            rospy.logwarn(str(self.latlontoutm_service_name)+" service not found!")
            return (None, None)

        try:
            latlontoutm_service = rospy.ServiceProxy(self.latlontoutm_service_name,
                                                     LatLonToUTM)
            gp = GeoPoint()
            if in_degrees:
                gp.latitude = lat
                gp.longitude = lon
            else:
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
        if self.no_service:
            rospy.logerr("The BT can not reach the latlon_to_utm service!")
            return []

        waypoints = []
        request_id = plandb.request_id
        plan_id = plandb.plan_id
        plan_spec = plandb.plan_spec

        if len(plan_spec.maneuvers) <= 0:
            rospy.logwarn("THERE WERE NO MANEUVERS IN THE PLAN! plan_id:{} (Does this vehicle know of your plan's maneuvers?)".format(plan_id))

        for plan_man in plan_spec.maneuvers:
            man_id = plan_man.maneuver_id
            maneuver = plan_man.maneuver
            man_name = maneuver.maneuver_name
            man_imc_id = maneuver.maneuver_imc_id
            # probably every maneuver has lat lon z in them, but just in case...
            # goto and sample are identical, with sample having extra "syringe" booleans...
            # cover_area is also the same, with extra Polygon field, that we can 
            # straight translate to more goto waypoints

            # GOTO, SAMPLE
            if man_imc_id in [imc_enums.MANEUVER_GOTO, imc_enums.MANEUVER_SAMPLE]:
                utm_x, utm_y = self.latlon_to_utm(maneuver.lat,
                                                  maneuver.lon,
                                                  -maneuver.z)
                if utm_x is None:
                    rospy.loginfo("Could not convert LATLON to UTM! Skipping point:{}".format((maneuver.lat, maneuver.lon, man_name)))
                    continue

                # EXTRA STUFF FOR SAMPLE
                extra_data = {}
                if man_imc_id == imc_enums.MANEUVER_SAMPLE:
                    extra_data = {'syringe0':maneuver.syringe0,
                                  'syringe1':maneuver.syringe1,
                                  'syringe2':maneuver.syringe2}


                # construct the waypoint object
                wp = Waypoint()
                wp.read_imc_maneuver(maneuver, utm_x, utm_y, extra_data)
                waypoints.append(wp)


            # COVER AREA
            elif man_imc_id == imc_enums.MANEUVER_COVER_AREA:
                if self.vehicle_localization_error_growth is None or \
                   self.coverage_swath is None:
                    rospy.loginfo("I do not know either the error growth or the swath, so I can not plan for coverage! Skipping the CoverArea maneuver!")
                    continue

                # always go to the point given in map as the first move.
                utm_x, utm_y = self.latlon_to_utm(maneuver.lat,
                                                  maneuver.lon,
                                                  -maneuver.z)


                utm_poly_points = [(utm_x, utm_y)]
                # this maneuver has an extra polygon with it
                # that we want to generate waypoints inside of
                # generate the waypoints here and add them as goto waypoints
                if len(maneuver.polygon) > 2:
                    rospy.loginfo("Generating rectangular coverage pattern")
                    utm_poly_points += [self.latlon_to_utm(polyvert.lat, polyvert.lon, -maneuver.z) for polyvert in maneuver.polygon]
                    coverage_points = self.generate_coverage_pattern(utm_poly_points)
                else:
                    rospy.loginfo("This polygon ({}) has too few polygons for a coverarea, it will be used as a simple waypoint!".format(man_id))
                    coverage_points = utm_poly_points


                for i,point in enumerate(coverage_points):
                    wp = Waypoint()
                    wp.read_maneuver(maneuver, point[0], point[1], {"poly":maneuver.polygon})
                    wp.wp.name = str(man_id) + "_{}/{}".format(i+1, len(coverage_points))
                    wp.imc_man_id = imc_enums.MANEUVER_GOTO
                    waypoints.append(wp)


            # UNIMPLEMENTED MANEUVER
            else:
                rospy.logwarn("SKIPPING UNIMPLEMENTED MANEUVER: id:{}, name:{}".format(man_imc_id, man_name))


        # sanity check
        if len(waypoints) <= 0:
            rospy.logerr("NO MANEUVERS IN MISSION PLAN!")

        return waypoints



    def generate_coverage_pattern(self, polygon):
        return create_coverage_path(polygon,
                                    self.coverage_swath,
                                    self.vehicle_localization_error_growth)


    def get_pose_array(self, flip_z=False):
        pa = PoseArray()
        pa.header.frame_id = self.plan_frame

        # add the rest of the waypoints
        for wp in self.waypoints:
            p = Pose()
            p.position.x = wp.x
            p.position.y = wp.y
            if flip_z:
                p.position.z = -wp.travel_depth
            else:
                p.position.z = wp.travel_depth
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



