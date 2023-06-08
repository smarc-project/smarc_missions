#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import rospy
import time
import math
import numpy as np
import py_trees as pt

from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from smarc_msgs.srv import LatLonToUTM, UTMToLatLon
from smarc_bt.msg import GotoWaypoint, MissionControl

# from coverage_planner import create_coverage_path
from dubins import create_dubins_path
from mission_log import MissionLog
import bb_enums

class Waypoint:
    def __init__(self, goto_waypoint = None, source=None):
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
            float64 arrival_heading
            bool use_heading
            string name
        """

        # the GotoWaypoint object from smarc_msgs.msg
        self.wp = goto_waypoint
        # use this to distinguish generated WPs and user-given WPs
        self.source = source


    def set_utm_from_latlon(self, lat_lon_to_utm_serv, set_frame=False):
        gp = GeoPoint()
        gp.latitude = self.wp.lat
        gp.longitude = self.wp.lon
        gp.altitude = 0
        res = lat_lon_to_utm_serv(gp)
        self.wp.pose.pose.position.x = res.utm_point.x
        self.wp.pose.pose.position.y = res.utm_point.y
        if set_frame:
            self.wp.pose.header.frame_id = 'utm'

    def set_latlon_from_utm(self, utm_to_lat_lon_serv, set_frame=False):
        p = Point()
        p.x = self.x
        p.y = self.y
        p.z = -self.wp.travel_depth
        res = utm_to_lat_lon_serv(p)
        self.wp.lat = res.lat_lon_point.latitude
        self.wp.lon = res.lat_lon_point.longitude
        if set_frame:
            self.wp.pose.header.frame_id = 'latlon'

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

    def __str__(self):
        s = 'Man: {}'.format(self.wp)
        return s



class MissionPlan:
    state_names = [
        "RUNNING",
        "STOPPED",
        "PAUSED",
        "EMERGENCY",
        "RECEIVED",
        "COMPLETED"]
    def __init__(self,
                 auv_config,
                 mission_control_msg = None
                 ):
        """
        A container object to keep things related to the mission plan.
        best way to use is to construct a mission_control_msg
        or pass a list of Waypoint objects in waypoints
        """
        # uint8 FB_RUNNING=0
        # uint8 FB_STOPPED=1
        # uint8 FB_PAUSED=2
        # uint8 FB_EMERGENCY=3
        # uint8 FB_RECEIVED=4
        self.state = MissionControl.FB_STOPPED

        # used to report when the mission was received
        self.creation_time = time.time()
        self._config = auv_config
        self.plan_id = "Unnamed plan - {}".format(self.creation_time)
        self.hash = ""
        self.timeout = -1
        self.mission_start_time = None
        # keep track of which waypoint we are going to
        # start at -1 to indicate that _we are not going to any yet_
        self.current_wp_index = -1
        # and finally read from the mission control message all
        # the fields above
        self.waypoints = []
        if mission_control_msg is not None:
            self._read_mission_control(mission_control_msg)

        # A track recording, in case this is a useful mission
        # Need to do this after read_mission_control otherwise
        # it wont know the name of the mission
        self.track = MissionLog(self)


    def tick(self):
        """
        A general update function to call for the mission plan
        Useful for things that might rely on mission state
        AND are not event-driven
        """
        # Decide if we should be recording the state of the vehicle
        # given the state of the mission plan
        # We want to start a recording when the mission starts
        # and end it when its stopped for whatever reason
        if self.track.recording:
            self.track.record()




    def _change_state(self, new_state):
        if self.state == MissionControl.FB_EMERGENCY:
            rospy.logwarn("Mission in emergency state! Not changing that!")
            return
        if new_state == self.state:
            return

        a = MissionPlan.state_names[self.state]
        b = MissionPlan.state_names[new_state]
        rospy.loginfo("{} -> {}".format(a,b))
        self.state = new_state


    def _read_mission_control(self, msg):
        """
        read the waypoints off a smarc_bt/MissionControl message
        and set our plan_id from its name
        """
        self.plan_id = msg.name
        self.hash = msg.hash
        self.timeout = msg.timeout
        self.waypoints = []
        ll_to_utm_serv = self._get_latlon_to_utm_service()
        for wp_msg in msg.waypoints:
            wp = Waypoint(goto_waypoint = wp_msg)
            # also make sure they are in utm
            wp.set_utm_from_latlon(ll_to_utm_serv, set_frame=True)
            self.waypoints.append(wp)

        self._change_state(MissionControl.FB_RECEIVED)
        rospy.loginfo("Got mission: name:{}, timeout:{}, num wps:{}, hash:{}".format(
            self.plan_id,
            self.timeout,
            len(self.waypoints),
            self.hash))


    def __str__(self):
        s = self.plan_id+':\n'
        for wp in self.waypoints:
            s += "->"+wp.wp.name
        return s

    def start_mission(self):
        self.mission_start_time = time.time()
        self.current_wp_index = 0
        rospy.loginfo("{} Started".format(self.plan_id))
        self._change_state(MissionControl.FB_RUNNING)
        self.track.start_recording()

    def pause_mission(self):
        rospy.loginfo("{} Paused".format(self.plan_id))
        self._change_state(MissionControl.FB_PAUSED)
        self.track.pause_recording()

    def continue_mission(self):
        rospy.loginfo("{} Continueing".format(self.plan_id))
        self._change_state(MissionControl.FB_RUNNING)
        self.track.continue_recording()

    def stop_mission(self):
        self.current_wp_index = -1
        rospy.loginfo("{} Stopped".format(self.plan_id))
        self._change_state(MissionControl.FB_STOPPED)
        self.track.stop_recording()

    def complete_mission(self):
        rospy.loginfo("{} Completed".format(self.plan_id))
        self._change_state(MissionControl.FB_COMPLETED)
        self.track.complete_recording()

    def emergency(self):
        self.current_wp_index = -1
        rospy.logwarn("{} EMERGENCY".format(self.plan_id))
        self._change_state(MissionControl.FB_EMERGENCY)
        self.track.stop_recording()

    def time_remaining(self):
        if self.mission_start_time is None:
            return -1

        runtime = time.time() - self.mission_start_time
        remaining = self.timeout - runtime
        return remaining

    def timeout_reached(self):
        if self.state == MissionControl.FB_EMERGENCY:
            return True

        if self.state == MissionControl.FB_RUNNING:
            if self.time_remaining() <= 0:
                self.emergency()
                rospy.logwarn("{} TIMEOUT".format(self.plan_id))
                return True

        return False


    def visit_wp(self):
        """ call this when you finish going to the wp"""
        if self.state == MissionControl.FB_RUNNING:
            self.current_wp_index += 1

        if self.current_wp_index >= len(self.waypoints):
            # we went tru all wps, we're done
            self._change_state(MissionControl.FB_COMPLETED)


    def get_current_wp(self, source=None):
        """
        pop a wp from the remaining wps and return it
        """
        if self.state == MissionControl.FB_RUNNING:
            wp = self.waypoints[self.current_wp_index]
            if source != None:
                rospy.loginfo("Current wp {} acquired from plan ({})".format(wp.wp.name, source))
            return wp

        return None


    def latlon_to_utm(self,
                      lat,
                      lon,
                      z,
                      in_degrees=False):

        serv = self._get_latlon_to_utm_service()
        if serv is None:
            return (None, None)

        gp = GeoPoint()
        if in_degrees:
            gp.latitude = lat
            gp.longitude = lon
        else:
            gp.latitude = np.degrees(lat)
            gp.longitude = np.degrees(lon)
        gp.altitude = z
        res = serv(gp)
        return (res.utm_point.x, res.utm_point.y)


    def _get_latlon_to_utm_service(self):
        bb = pt.blackboard.Blackboard()
        s = bb.get(bb_enums.LLTOUTM_SERVICE_NAME)
        rospy.wait_for_service(s, timeout=1)
        latlontoutm_service = rospy.ServiceProxy(s, LatLonToUTM)
        return latlontoutm_service


    def _get_utm_to_latlon_service(self):
        bb = pt.blackboard.Blackboard()
        s = bb.get(bb_enums.UTMTOLL_SERVICE_NAME)
        rospy.wait_for_service(s, timeout=1)
        utmtolatlon_service = rospy.ServiceProxy(s, UTMToLatLon)
        return utmtolatlon_service


    def generate_dubins(self, turning_radius=3, step=None):
        if step is None:
            step = turning_radius + 0.5
        rospy.loginfo("Converting to dubins plan with radius {} and step {}".format(turning_radius, step))
        bb = pt.blackboard.Blackboard()
        vehicle = bb.get(bb_enums.VEHICLE_STATE)
        # only give waypoints that are not generated by anything
        # in case someone clicks "plan" 10 times
        waypoints = []
        for wp in self.waypoints:
            if wp.source is None:
                waypoints.append(wp)
        dubins_waypoints = create_dubins_path(vehicle,
                                              waypoints,
                                              step,
                                              turning_radius)
        utm_to_ll_serv = self._get_utm_to_latlon_service()
        for wp in dubins_waypoints:
            wp.set_latlon_from_utm(utm_to_ll_serv, set_frame=False)

        self.waypoints = dubins_waypoints
        rospy.loginfo("The plan now has {} WPs".format(len(self.waypoints)))