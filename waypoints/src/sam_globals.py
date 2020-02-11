#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


MINIMUM_PLAN_STR_LEN = 135 # somehow Chris found this number, i'll reuse this

CURRENT_PLAN_ACTION = 'current_plan_action'
LAST_PLAN_ACTION_FEEDBACK = 'last_plan_action_feedback'

# DO NOT PUT SLASHES BEFORE TOPIC NAMES, that puts them in 'root' in ros's eyes.
PLAN_TOPIC = 'plan_db'
#  SAM_GPS_TOPIC = 'core/gps'
ESTIMATED_STATE_TOPIC = 'estimated_state'
PLAN_CONTROL_STATE_TOPIC = 'plan_control_state'
ABORT_TOPIC = 'abort'

#TODO a better way...
BASE_LINK = '/sam/base_link'

# TODO remove
#  GPS_FIX_BB = 'gps_fix'
#  UTM_BAND_BB = 'utm_band'
#  UTM_ZONE_BB = 'utm_zone'

# TODO fill in from data ingestion tree
ABORT_BB = 'abort'
DEPTH_BB = 'depth'
ALTITUDE_BB = 'altitude'
MISSION_PLAN_STR_BB = 'plan_str'
MISSION_PLAN_OBJ_BB = 'misison_plan'

DEPTH_TOPIC = 'ctrl/depth_feedback'
ALTITUDE_TOPIC = ''

SAM_MAX_DEPTH = 5
SAM_MIN_ALTITUDE = 2
