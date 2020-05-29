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
        self.DEPTH_TOPIC = 'ctrl/depth_feedback'
        self.ALTITUDE_TOPIC = 'core/dvl'
        self.LEAK_TOPIC = 'core/leak_fb'
        self.GPS_FIX_TOPIC = 'core/gps'
        self.CAMERA_DETECTION_TOPIC = 'detection/poi_down'

        # actions and services
        self.ACTION_NAMESPACE = 'ctrl/wp_depth_action_planner'
        self.EMERGENCY_ACTION_NAMESPACE = 'ctrl/emergency_surface_action'
        # this can be set to None to disable the use of a path planner
        # the robot will be given the user generated waypoints to follow in that case
        self.PATH_PLANNER_NAME = '/interp1d'

        # tf frame names
        self.BASE_LINK = 'base_link'
        self.UTM_LINK = 'utm'
        self.LOCAL_LINK = 'map'
        self.POI_DETECTOR_LINK = 'sam/camera_down_link'

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
        # how many ticks to run emergency action before we give up
        # on the current wp and skip it
        self.EMERGENCY_TRIALS_BEFORE_GIVING_UP = 10

    def __str__(self):
        s = 'AUV_CONFIG:\n'
        for k,v in vars(self).iteritems():
            s += str(k)+':'+str(v) +'\n'
        s += '********\n'
        return s


