#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


ABORT = 'abort'
LEAK = 'leak'

DEPTH = 'depth'
ALTITUDE = 'altitude'

#TODO remove this msg/obj differentiation...
MISSION_PLAN_MSG = 'plan_msg'
MISSION_PLAN_OBJ = 'misison_plan'
PLAN_CONTROL_MSG = 'plan_control_msg'

UTM_BAND = 'utm_band'
UTM_ZONE = 'utm_zone'

WORLD_ROT = 'world_rot'
WORLD_TRANS = 'world_trans'
BASE_LINK = 'base_link'

IMC_STATE = 'imc_state'

CURRENT_PLAN_ACTION = 'current_plan_action'
LAST_PLAN_ACTION_FEEDBACK = 'last_plan_action_feedback'
# set this from any action that might return RUNNING.
# useful for feedback purposes
CURRENTLY_RUNNING_ACTION = 'currently_running_action'


