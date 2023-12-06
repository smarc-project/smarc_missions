#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

import rospy, time, os
import numpy as np
import json

from mission_plan import MissionPlan
import bb_enums

from smarc_bt.msg import MissionControl, BTCommand

from rospy_message_converter import json_message_converter

class NoderedHandler(object):
    """
    A parallel to the neptus handler, that handles the
    communication with the nodered interface
    this includes things like start/stop/pause plan
    asnwer questions about the plan and set up mission plans
    """
    def __init__(self,
                 auv_config,
                 vehicle,
                 blackboard):

        self._vehicle = vehicle
        self._config = auv_config
        self._bb = blackboard

        self._last_received_mc_msg = None
        self._mission_control_sub = rospy.Subscriber(self._config.MISSION_CONTROL_TOPIC,
                                                     MissionControl,
                                                     self._mission_control_cb,
                                                     queue_size=1)

        self._mc_msg = MissionControl()
        self._mission_control_pub = rospy.Publisher(self._config.MISSION_CONTROL_TOPIC,
                                                    MissionControl,
                                                    queue_size=1)

        self._last_received_btc_msg = None
        self._bt_command_sub = rospy.Subscriber(self._config.BTCOMMAND_TOPIC,
                                                BTCommand,
                                                self._bt_command_cb,
                                                queue_size=1)

        self._btc_msg = BTCommand()
        self._bt_command_pub = rospy.Publisher(self._config.BTCOMMAND_TOPIC,
                                                    BTCommand,
                                                    queue_size=1)

    def _mission_control_cb(self, msg):
        self._last_received_mc_msg = msg

    def _bt_command_cb(self, msg):
        self._last_received_btc_msg = msg

    def _publish_current_plan(self):
        # simply publish the current plan at every tick
        self._mc_msg.command = MissionControl.CMD_IS_FEEDBACK
        mission_plan = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        if mission_plan is None:
            self._mc_msg.name = "No plan"
            self._mc_msg.plan_state = MissionControl.FB_STOPPED
            self._mc_msg.waypoints = []
        else:
            self._mc_msg.name = mission_plan.plan_id
            self._mc_msg.plan_state = mission_plan.state
            # mission plan should contain a list of waypoint objects that each contain
            # a GotoWaypoint object called wp
            self._mc_msg.waypoints = [wp.wp for wp in mission_plan.waypoints]
            # also inherit the hash given in the message for feedback
            self._mc_msg.hash = mission_plan.hash

        self._mission_control_pub.publish(self._mc_msg)


    def _command_matches_known_mission(self, msg):
        current_mission = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        if current_mission is None:
            rospy.loginfo("Mission start/stop/pause given but there is no mission?")
            return False
        if current_mission.plan_id != msg.name:
            s = "Command given for: {}, but we got: {}, stopping current mission!".format(msg.name, current_mission.plan_id)
            rospy.loginfo(s)
            current_mission.stop_mission()
            return False
        return True


    def _load_mission(self, msg):
        path = os.path.expanduser(self._config.MISSION_PLAN_STORAGE_FOLDER)
        filename = os.path.join(path, msg.name+".json")
        with open(filename, 'r') as f:
            j = f.read()
            mission_control_msg = json_message_converter.convert_json_to_ros_message("smarc_msgs/MissionControl", j)

        rospy.loginfo("Loaded mission {} from file!".format(msg.name))
        return mission_control_msg


    def _save_mission(self, msg):
        if "TEST--" in msg.name:
            rospy.loginfo("Test mission, not saving")
            return

        path = os.path.expanduser(self._config.MISSION_PLAN_STORAGE_FOLDER)
        if not os.path.exists(path):
            os.makedirs(path)

        filename = os.path.join(path, msg.name+".json")
        with open(filename, 'w') as f:
            j = json_message_converter.convert_ros_message_to_json(msg)
            f.write(j)
            rospy.loginfo("Wrote mission {}".format(filename))



    def _set_plan(self, msg):
        """
        The message might contain a proper complete plan
        or just the hash of a plan.
        If its a proper mission:
            - Check if a saved mission with the same name exists
                - over-write it if it exists
                    - Save the msg object directly since its already serialized and such, with msg.name as its filename
            - If no same-name saved mission exists, save this one with its hash
        If it is NOT a proper mission = no waypoints then we need to check if we have a saved
        mission with the same hash. If so, we load it, if not, do nothing.
            - Check the msg.hash and msg.name fields, everything else can be empty for this
            - Purpose: Super-low-bandwidth mission selection
        """
        if(msg.name != "" and msg.hash != "" and len(msg.waypoints) == 0):
            # try to load a mission from file
            try:
                loaded_msg = self._load_mission(msg)
            except:
                rospy.logwarn("Could not load mission with name: {}".format(msg.name))
                return

            # okay, got a mission message, but does it have the same hash
            # as the request?
            if loaded_msg.hash == msg.hash:
                msg = loaded_msg
            else:
                rospy.logwarn("A plan with this name exists, and has a different hash: {}".format(msg.name))
                return

        # we either got a proper full mission message
        # or the loaded mission message is good to use
        self._save_mission(msg)
        new_plan = MissionPlan(auv_config = self._config,
                               mission_control_msg = msg)

        self._bb.set(bb_enums.MISSION_PLAN_OBJ, new_plan)
        rospy.loginfo("New mission {} set!".format(msg.name))


    def _publish_fb_dict(self, fb):
        fb_json = json.dumps(fb)
        self._btc_msg.msg_type = BTCommand.TYPE_FB
        self._btc_msg.fb_json = fb_json
        self._bt_command_pub.publish(self._btc_msg)


    def _handle_bt_command(self):
        if self._last_received_btc_msg is None:
            return
        msg = self._last_received_btc_msg

        if msg.msg_type == BTCommand.TYPE_FB:
            # just ignore feedback
            return

        def _get_filepath():
            path = os.path.expanduser(self._config.MISSION_LOG_FOLDER)
            if arg is None:
                fb = {"error":"No arg given"}
                self._publish_fb_dict(fb)
                return None
            filepath = os.path.join(path, arg)
            return filepath


        # turn the cmd_json thing into a dict so we can parse it easy
        cmd_json = json.loads(msg.cmd_json)
        cmd = cmd_json['cmd']
        try:
            arg = cmd_json['arg']
        except:
            arg = None
        rospy.loginfo_throttle_identical(10, "Got command: {}, arg: {}".format(cmd, arg))
        if cmd == "request_track_list":
            path = os.path.expanduser(self._config.MISSION_LOG_FOLDER)
            files = sorted(os.listdir(path))
            files_sizes = []
            for file in files:
                filepath = os.path.join(path, file)
                stats = os.stat(filepath)
                filesize = stats.st_size/1024
                files_sizes.append({"filename":file, "filesize":filesize})
            fb = {"track_list":files_sizes}
            self._publish_fb_dict(fb)

        if cmd == "download_track":
            filepath = _get_filepath()
            if filepath is None: return
            with open(filepath, 'r') as f:
                track = f.read()
            fb = {"track":track}
            self._publish_fb_dict(fb)

        if cmd == "delete_track":
            filepath = _get_filepath()
            if filepath is None: return
            os.remove(filepath)
            fb = {"result":"Deleted {}".format(arg)}
            self._publish_fb_dict(fb)

        if cmd == "size_track":
            filepath = _get_filepath()
            if filepath is None: return
            stat = os.stat(filepath)
            fb = {"result":"Size= {} Bytes".format(stat.st_size)}
            self._publish_fb_dict(fb)

        if cmd == "plan_dubins":
            try:
                turning_radius = float(arg)
            except:
                fb = {"result":"Turning radius was bad: {}".format(arg)}
                self._publish_fb_dict(fb)
                return

            mplan = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
            if mplan is None:
                fb = {"result":"Mission plan was None"}
                self._publish_fb_dict(fb)
                return

            mplan.generate_dubins(turning_radius=turning_radius)
            fb = {"result":"Mission plan is dubins"}
            self._publish_fb_dict(fb)
            self._publish_current_plan()




    def _handle_mission_control(self):
        if self._last_received_mc_msg is None:
            return

        # there might be a command from nodered, check it
        msg = self._last_received_mc_msg

        if msg.command == MissionControl.CMD_IS_FEEDBACK:
            # just silently ignore these
            return

        if msg.command == MissionControl.CMD_SET_PLAN:
            rospy.loginfo("SET PLAN command received")
            self._set_plan(msg)
            return

        if msg.command == MissionControl.CMD_REQUEST_FEEDBACK:
            rospy.loginfo("REQ FEEDBACK command received")
            self._publish_current_plan()
            return

        # other commands rely on the current mission in ways
        current_mission = self._bb.get(bb_enums.MISSION_PLAN_OBJ)
        # first order of buisness, if its an emergency, or a mission-indep message
        # handle that
        if msg.command == MissionControl.CMD_EMERGENCY:
            rospy.loginfo("EMERGENCY command received")
            current_mission.emergency()
            rospy.logwarn("Aborted")
            return


        # anything else, we need to check if the command
        # actually matches what we have already
        if not self._command_matches_known_mission(msg):
            rospy.logwarn("Received message ({}) doesn't match the current plan! Ignoring it!".format(msg))
            return

        if msg.command == MissionControl.CMD_START:
            rospy.loginfo("START command received")
            if current_mission.state in [MissionControl.FB_STOPPED, MissionControl.FB_RECEIVED]:
                current_mission.start_mission()
                return

            if current_mission.state == MissionControl.FB_PAUSED:
                current_mission.continue_mission()
                return

        if msg.command == MissionControl.CMD_STOP:
            rospy.loginfo("STOP command received")
            current_mission.stop_mission()
            return

        if msg.command == MissionControl.CMD_PAUSE:
            rospy.loginfo("PAUSE command received")
            current_mission.pause_mission()
            return


    def tick(self):
        self._publish_current_plan()
        self._handle_mission_control()
        self._handle_bt_command()


