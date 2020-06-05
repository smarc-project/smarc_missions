#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


ABORT = 'abort'
LEAK = 'leak'

DEPTH = 'depth'
ALTITUDE = 'altitude'

MISSION_PLAN_OBJ = 'misison_plan'
PLAN_IS_GO = 'plan_is_go'
MANEUVER_ACTIONS = 'maneuver_actions'

UTM_BAND = 'utm_band'
UTM_ZONE = 'utm_zone'

CURRENT_LATITUDE = 'lat'
CURRENT_LONGITUDE ='lon'
WORLD_ROT = 'world_rot'
WORLD_TRANS = 'world_trans'
LOCATION_POINT_STAMPED = 'loc_ps'
BASE_LINK = 'base_link'
POI_POINT_STAMPED = 'poi_ps'

IMC_STATE = 'imc_state'

CURRENT_PLAN_ACTION = 'current_plan_action'
LAST_PLAN_ACTION_FEEDBACK = 'last_plan_action_feedback'
# set this from any action that might return RUNNING.
# useful for feedback purposes
CURRENTLY_RUNNING_ACTION = 'currently_running_action'
ENABLE_AUTONOMY = 'enable_autonomy'

TREE_TIP_NAME = 'tree_tip_name'
TREE_TIP_STATUS = 'tree_tip_status'

