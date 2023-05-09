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

        self.name = mission_plan.plan_id
        self.start = mission_plan.mission_start_time
        self.end = None
        self.robot_name = self.vehicle.robot_name
        self.recording = False
        self.locked = False
        self.data = []


        utm_zone = rospy.get_param(rospy.search_param("utm_zone"), "NOZONE")
        utm_band = rospy.get_param(rospy.search_param("utm_band"), "NOBAND")
        self.utm_z = "{}{}".format(utm_zone,utm_band)

        t = datetime.datetime.now()
        tstr = t.strftime("%Y-%m-%d_%H-%M")
        filename = tstr+"_" + self.name + ".json"
        folder = mission_plan._config.MISSION_LOG_FOLDER
        folder = os.path.expanduser(folder)
        if not os.path.exists(folder):
            os.makedirs(folder)
        self.filepath = os.path.join(folder, filename)

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
        """
        v = self.vehicle

        base = {
            "name":self.robot_name,
            "lat":v.position_latlon[0],
            "lon":v.position_latlon[1],
            "utm_x":v.position_utm[0],
            "utm_y":v.position_utm[1],
            "utm_z":self.utm_z, # this is static in a deployment
            "heading": RADTODEG * (math.pi/2 - v.orientation_rpy[2]), # same thing in nodered
            "roll": v.orientation_rpy[0],
            "pitch": v.orientation_rpy[1],
            "depth": v.depth,
            "altitude": v.altitude,
            "vbs":v.vbs,
            "lcg":v.lcg,
            "tcg":None,
            "t1":v.t1,
            "t2":v.t2,
            "batt_v":v.batt_v,
            "batt_percent":v.batt_percent,
            "gps_lat": v.raw_gps_obj.latitude,
            "gps_lon": v.raw_gps_obj.longitude
        }
        tip = self.bb.get(bb_enums.TREE_TIP)
        plan = self.bb.get(bb_enums.MISSION_PLAN_OBJ)
        status_strings = {pt.Status.SUCCESS:"success",
                          pt.Status.FAILURE:"failure",
                          pt.Status.RUNNING:"running",
                          pt.Status.INVALID:"invalid"}

        current_action = self.bb.get(bb_enums.CURRENT_PLAN_ACTION)
        current_wp = None
        if current_action is not None:
            current_wp = current_action.wp

        bt = {
            "tip":{
                "name":tip.name,
                "message":tip.feedback_message,
                "status":status_strings.get(tip.status, "unknown")
            },
            "algae_farm_enable":self.bb.get(bb_enums.ALGAE_FOLLOW_ENABLE),
            "live_wp_enable":self.bb.get(bb_enums.LIVE_WP_ENABLE),
            "gui_wp_enable":self.bb.get(bb_enums.GUI_WP_ENABLE),
            "current_wp": current_wp,
            "current_plan":{
                "name":plan.plan_id,
                "hash":plan.hash,
                "timeout":plan.timeout,
                "command":-1,
                "plan_state":plan.state,
                "feedback_str":"",
                "waypoints":[]},
            "last_heartbeat":self.bb.get(bb_enums.LAST_HEARTBEAT_TIME)
            }

        r = {'last_update':time.time(),
             'base':base,
             'bt':bt,
             'annotation':annotation}
        self.data.append(r)

    def save(self):
        # nodered saves a JSON, so do we.
        # straight up copying the saving function from JS here
        # yes, this is REALLY BAD. 
        # i know.
        less = {}
        less['name'] = self.name
        less['start'] = self.start
        less['end'] = self.end
        less['robot_name'] = self.robot_name
        less['data_columns'] = [
            "update_time",
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
            "current_wp",
            "current_plan_name",
            "bt_heartbeat_time",
        ]
        less['current_wp_columns'] = [
            "name",
            "utm_x",
            "utm_y",
            "goal_tolerance",
            "z_control_mode",
            "travel_altitude",
            "travel_depth",
            "speed_control_mode",
            "travel_rpm",
            "travel_speed",
            "lat",
            "lon",
            ]
        less['data'] = []
        for d in self.data:
            c = d['bt']['current_wp']
            cwp = []
            if c is not None:
                cwp = [
                    c.name,
                    c.pose.pose.position.x,
                    c.pose.pose.position.y,
                    c.goal_tolerance,
                    c.z_control_mode,
                    c.travel_altitude,
                    c.travel_depth,
                    c.speed_control_mode,
                    c.travel_rpm,
                    c.travel_speed,
                    c.lat,
                    c.lon
                    ] 
            b = d['base']
            tip = d['bt']['tip']
            line = [
                d['last_update'],
                d['annotation'],
                b['lat'],
                b['lon'],
                b["utm_x"],
                b["utm_y"],
                b["utm_z"],
                b["heading"],
                b["roll"],
                b["pitch"],
                b["depth"],
                b["altitude"],
                b["vbs"],
                b["lcg"],
                b["tcg"],
                b["t1"],
                b["t2"],
                b["batt_v"],
                b["batt_percent"],
                b["gps_lat"],
                b["gps_lon"],
                tip['name'],
                tip['status'],
                cwp,
                d['bt']['current_plan']['name'],
                d['bt']['last_heartbeat']
            ]
            less['data'].append(line)

        with open(self.filepath, 'w') as f:
            json.dump(less, f)
            rospy.loginfo("Dumped log into {}".format(self.filepath))

                
