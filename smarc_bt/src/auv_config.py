#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

"""
A simple config object for an AUV, to make sure that
the fields are common.
"""

import rospy

class AUVConfig(object):
    """
    Base config object, with default values for SAM.
    """
    def __init__(self):
        self.robot_name = 'sam'

        # topics
        self.DEPTH_TOPIC = 'ctrl/odom_listener/depth_feedback'
        self.ALTITUDE_TOPIC = 'core/dvl'
        self.LEAK_TOPIC = 'core/leak'
        self.GPS_FIX_TOPIC = 'core/gps'
        self.CAMERA_DETECTION_TOPIC = 'detection/poi_down'
        self.PATH_TOPIC = 'ctrl/planned_path'
        self.PLAN_VIZ_TOPIC = 'viz/mission_waypoints'

        self.EMERGENCY_TOPIC = 'abort'
        self.HEARTBEAT_TOPIC = 'heartbeat'
        self.MISSION_COMPLETE_TOPIC = 'mission_complete'

        # actions and services
        self.ACTION_NAMESPACE = 'ctrl/goto_waypoint'
        self.EMERGENCY_ACTION_NAMESPACE = 'ctrl/emergency_surface_action'
        self.FOLLOW_ACTION_NAMESPACE = 'ctrl/leader_follower_action'
        self.START_STOP_DVL_NAMESPACE = 'core/toggle_dvl'
        # this can be set to None to disable the use of a path planner
        # the robot will be given the user generated waypoints to follow in that case
        self.PATH_PLANNER_NAME = 'None'

        # tf frame names
        self.BASE_LINK = self.robot_name+'/base_link'
        self.UTM_LINK = 'utm'
        self.LOCAL_LINK = 'map'
        # this should be given as /sam/camera_down_link  or sam_1/camera_down_link etc.
        self.POI_DETECTOR_LINK = self.robot_name+'/camera_down_link'
        # some other link that is not THIS robot
        self.LEADER_LINK = 'sam_1/base_link'
        self.ENABLE_LEADER_FOLLOWER = True

        # imc related stuff, most likely never changes
        self.PLANDB_TOPIC = 'imc/plandb'
        self.PLAN_CONTROL_TOPIC = 'imc/plan_control'
        self.ESTIMATED_STATE_TOPIC = 'imc/estimated_state'
        self.PLAN_CONTROL_STATE_TOPIC = 'imc/plan_control_state'
        self.VEHICLE_STATE_TOPIC = 'imc/vehicle_state'
        self.ABORT_TOPIC = 'imc/abort'

        # hard values
        self.MAX_DEPTH = 20
        self.MIN_ALTITUDE = 5
        self.ABSOLUTE_MIN_ALTITUDE = 1
        # how many ticks to run emergency action before we give up
        # on the current wp and skip it
        # in ticks
        self.EMERGENCY_TRIALS_BEFORE_GIVING_UP = 30
        self.MIN_DISTANCE_TO_LEADER = 5
        # in seconds, dont spam the dvl service
        self.DVL_COOLDOWN = 0.5
        # how deep do we want to be to run the dvl
        # we wanna shut off the dvl on the surface
        self.DVL_RUNNING_DEPTH = 0.55

        # in meters
        self.WAYPOINT_TOLERANCE = 1.5

        # set global rosparams that _must_ be there to None
        # i should find a better way of doing this but eh
        self.UTM_ZONE = None
        self.UTM_BAND = None

    def __str__(self):
        s = 'AUV_CONFIG:\n'
        for k,v in vars(self).iteritems():
            s += str(k)+':'+str(v) +'\n'
        s += '********\n'
        return s


    def generate_launch_file(self, catkin_ws_path):
        def make_arg(name, default):
            return '\t<arg name="{}" default="{}" />\n'.format(name.lower(), default)

        def make_param(name):
            return '\t\t<param name="{}" value="$(arg {})" />\n'.format(name.lower(), name.lower())

        args_part = ''
        params_part ='\n\n\t<node name="sam_bt" pkg="smarc_bt" type="sam_bt.py" output="screen" ns="$(arg robot_name)">\n'
        for k,v in vars(self).iteritems():
            args_part += make_arg(k,v)
            params_part += make_param(k)

        params_part += '\t</node>\n'

        bt_launch_path = 'catkin_ws/src/smarc_missions/smarc_bt/launch/bt_sam.launch'
        with open(catkin_ws_path+bt_launch_path, 'w+') as f:
            f.write('<!-- THIS LAUNCH FILE WAS AUTO-GENERATED FROM, DO NOT MODIFY src/auv_config.py -->\n\n')
            f.write('<launch>\n')
            f.write(args_part)
            f.write(params_part)
            f.write('</launch>\n')


        print("Generated default launch file at {}".format(catkin_ws_path+bt_launch_path))
        print("You might need to restart the ros mon instance to read the new launch file")


    def read_rosparams(self):
        for k,v in vars(self).iteritems():
            if v is not None:
                param_name = '~'+k.lower()
            else:
                param_name = k.lower()
            rosparam_v = rospy.get_param(param_name, None)
            if rosparam_v is not None:
                self.__dict__[k] = rosparam_v

            assert not (v is None and rosparam_v is None), "A required global rosparam ({}) is not set!".format(k)



