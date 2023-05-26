#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import rospy
import time, datetime
import math
import py_trees as pt
import os
import json

import bb_enums

RADTODEG = 360 / (math.pi * 2)

class MissionLog(object):
    def __init__(self, mission_plan):
        """
        We want to mimic the same structure that nodered uses
        function new_track(){
            now = Date.now()
            t = new Date(now).toTimeString().slice(0,8)
            return {
                // name should be
                // start time - robot_name - mission name
                name:"Track-"+ t,
                start: null,
                end: null,
                robot_name: null,
                recording: false,
                locked: false,
                data: []
            }
        }
        """
        self.bb = pt.blackboard.Blackboard()
        self.vehicle = self.bb.get(bb_enums.VEHICLE_STATE)

        t = datetime.datetime.now()
        tstr = t.strftime("%Y-%m-%d_%H-%M")
        self.name = tstr+"_" + mission_plan.plan_id + ".json"
        self.start = mission_plan.mission_start_time
        self.end = None
        self.robot_name = self.vehicle.robot_name
        self.recording = False
        self.locked = False
        self.data = []

        self.last_save = None
        self.save_interval_secs = mission_plan._config.MISSION_LOG_AUTOSAVE_INTERVAL_SECS


        utm_zone = rospy.get_param(rospy.search_param("utm_zone"), "NOZONE")
        utm_band = rospy.get_param(rospy.search_param("utm_band"), "NOBAND")
        self.utm_z = "{}{}".format(utm_zone,utm_band)

        folder = mission_plan._config.MISSION_LOG_FOLDER
        folder = os.path.expanduser(folder)
        if not os.path.exists(folder):
            os.makedirs(folder)
        self.filepath = os.path.join(folder, self.name)

        self.track_columns = [
            "timestamp",
            "annotation",
            "lat",
            "lon",
            "utm_x",
            "utm_y",
            "utm_z",
            "heading",
            "roll",
            "pitch",
            "depth",
            "altitude",
            "vbs",
            "lcg",
            "tcg",
            "t1",
            "t2",
            "batt_v",
            "batt_percent",
            "gps_lat",
            "gps_lon",
            "bt_tip_name",
            "bt_tip_status",
            "plan_name",
            "wp_name",
            "wp_lat",
            "wp_lon",
            "wp_utm_x",
            "wp_utm_y",
            "wp_goal_tolerance",
            "wp_z_control_mode",
            "wp_travel_altitude",
            "wp_travel_depth",
            "wp_speed_control_mode",
            "wp_travel_rpm",
            "wp_travel_speed"
        ]

    ############################
    # These mirror the actions of a mission plan, except emergency which is
    # not different than stopping for a track
    ############################
    def start_recording(self):
        self.recording = True
        self.start = time.time()
        self.record(annotation="recording_start")
        self.save()

    def stop_recording(self):
        self.recording = False
        self.end = time.time()
        self.record(annotation="recording_stop")
        self.save()

    def pause_recording(self):
        self.recording = False
        self.record(annotation="recording_pause")
        self.save()

    def continue_recording(self):
        self.recording = True 
        self.record(annotation="recording_continue")
        self.save()

    def complete_recording(self):
        self.recording = False
        self.end = time.time()
        self.record(annotation="recording_complete")
        self.save()

    def record(self, annotation=None):
        """
        Mimic the structure of nodered, hopefully we dont forget to update this :D
        I COULD make nodered send a template to fill in and all that... but too much work
        I dont expect this to change very often if at all
        lat/lons are rounded to 6 decimals
        utms to 2 decimals
        angles to 2 decimals
        """
        v = self.vehicle
        tip = self.bb.get(bb_enums.TREE_TIP)
        plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        status_strings = {pt.Status.SUCCESS:"success",
                          pt.Status.FAILURE:"failure",
                          pt.Status.RUNNING:"running",
                          pt.Status.INVALID:"invalid"}


        if annotation is None:
            annotation = ""

        def xround(num, precision=0):
            try:
                return round(num, precision)
            except:
                return num

        record = [
            int(time.time()),
            annotation,
            xround(v.position_latlon[0], 6),
            xround(v.position_latlon[1], 6),
            xround(v.position_utm[0], 2),
            xround(v.position_utm[1], 2),
            self.utm_z,
            xround(RADTODEG * (math.pi/2 - v.orientation_rpy[2]), 2), #heading
            xround(v.orientation_rpy[0], 2),
            xround(v.orientation_rpy[1], 2),
            xround(v.depth, 2),
            xround(v.altitude, 2),
            xround(v.vbs, 2),
            xround(v.lcg, 2),
            None, #tcg
            v.t1,
            v.t2,
            xround(v.batt_v, 1),
            v.batt_percent,
            xround(v.raw_gps_obj.latitude, 6),
            xround(v.raw_gps_obj.longitude, 6),
            tip.name,
            status_strings.get(tip.status, "unknown")[0],
            plan.plan_id
            ]

        current_action = self.bb.get(bb_enums.CURRENT_PLAN_ACTION)
        if current_action is not None:
            current_wp = current_action.wp
            record.extend([
                current_wp.name,
                xround(current_wp.lat, 6),
                xround(current_wp.lon, 6),
                xround(current_wp.pose.pose.position.x, 2),
                xround(current_wp.pose.pose.position.y, 2),
                current_wp.goal_tolerance,
                current_wp.z_control_mode,
                current_wp.travel_altitude,
                current_wp.travel_depth,
                current_wp.speed_control_mode,
                current_wp.travel_rpm,
                current_wp.travel_speed
            ])
        else:
            # all the current_wp stuff is None if there is no current action
            record.extend([None]*12)

        self.data.append(record)

        if self.last_save is None:
            self.save()
        elif time.time() - self.last_save > self.save_interval_secs:
            self.save()

    def save(self):
        # nodered saves a JSON, so do we.
        # straight up copying the saving function from JS here
        dict_track = {
            "name":self.name,
            "start":self.start,
            "end":self.end,
            "robot_name":self.robot_name,
            "recording":self.recording,
            "locked":self.locked,
            "data":self.data,
            "columns":self.track_columns
        }

        with open(self.filepath, 'w') as f:
            if not "TEST--" in dict_track["name"]:
                json.dump(dict_track, f)
                rospy.loginfo("Dumped log into {}".format(self.filepath))
            else:
                rospy.loginfo("Test mission, not dumping logs")

        self.last_save = time.time()

                
