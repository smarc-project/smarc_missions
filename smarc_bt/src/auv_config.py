#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

"""
A simple config object for an AUV, to make sure that
the fields are common.
"""


class AUVConfig(object):
    """
    Base config object, with default values for SAM.
    """
    def __init__(self, robot_name):
        self.robot_name = robot_name

        self.DEPTH_TOPIC = 'ctrl/depth_feedback'
        self.ALTITUDE_TOPIC = 'ctrl/altitude_feedback'
        self.LEAK_TOPIC = 'core/leak_fb'
        self.GPS_FIX_TOPIC = 'core/gps'

        self.ACTION_NAMESPACE = 'ctrl/wp_depth_action_planner'
        self.EMERGENCY_ACTION_NAMESPACE = 'ctrl/emergency_surface_action'

        self.BASE_LINK = 'base_link'
        self.UTM_LINK = 'world_utm'

        # imc related stuff, most likely never changes
        self.PLANDB_TOPIC = 'imc/plandb'
        self.PLAN_CONTROL_TOPIC = 'imc/plan_control'
        self.ESTIMATED_STATE_TOPIC = 'imc/estimated_state'
        self.PLAN_CONTROL_STATE_TOPIC = 'imc/plan_control_state'
        self.VEHICLE_STATE_TOPIC = 'imc/vehicle_state'
        self.ABORT_TOPIC = 'imc/abort'

        self.MAX_DEPTH = 5
        self.MIN_ALTITUDE = 2

    def __str__(self):
        s = 'AUV_CONFIG:\n'
        for k,v in vars(self).iteritems():
            s += str(k)+':'+str(v) +'\n'
        s += '********\n'
        return s




class SAMConfig(AUVConfig):
    def __init__(self):
        super(SAMConfig, self).__init__('sam')


class LOLOConfig(AUVConfig):
    def __init__(self):
        super(LOLOConfig, self).__init__('lolo')

        # example
        self.ACTION_NAMESPACE = 'ctrl/lolos_better_wp_follower_action'
        self.MAX_DEPTH = 20
        self.MIN_ALTITUDE = 1
