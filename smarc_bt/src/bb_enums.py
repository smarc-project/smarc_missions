#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

ROBOT_NAME = 'robot_name'

VEHICLE_STATE = 'vehicle_state'

ABORT = 'abort'
MAX_DEPTH = 'max_depth'
MIN_ALTITUDE = 'min_altitude'

MISSION_PLAN_OBJ = 'misison_plan'
MANEUVER_ACTIONS = 'maneuver_actions'

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

DVL_IS_RUNNING = 'dvl_is_running'

# to set once the BT is 'DONE' done. LIke, it wont want to
# set new waypoints or anything, and is just there chillin
# and handling stuff like neptus etc.
MISSION_FINALIZED = 'mission_finalized'

# coverage stuffs
# these are in the BB becse they could be dynamic
SWATH = 'swath'
LOCALIZATION_ERROR_GROWTH = 'loc_err_growth'

# Algae farm
BUOYS = 'buoys'
ALGAE_FOLLOW_ENABLE = 'algea_enable'
ALGAE_FOLLOW_WP = 'algae_follow_wp'

# Nacho's relocalization stuff
RELOC_ENABLE = 'reloc_enable'
RELOC_WP = 'reloc_wp'

# Goto action
WAYPOINT_TOLERANCE = 'wp_tolerance'


# Mission log object
MISSION_LOG_OBJ = 'mission_log'
MISSION_LOG_FOLDER = 'mission_logs_folder'
ENABLE_MANUAL_MISSION_LOG = 'enable_manual_mission_log'
MANUAL_MISSION_LOG_OBJ = 'manual_mission_log'

# lolo-specific
LOLO_ELEVATOR = 'lolo_elevator'
LOLO_ELEVON_PORT = 'lolo_elevon_port'
LOLO_ELEVON_STRB = 'lolo_elevon_strb'
LOLO_AFT_TANK = 'lolo_aft_tank'
LOLO_FRONT_TANK = 'lolo_front_tank'
LOLO_AFT_TANK_TARGET = 'lolo_aft_tank_target'
LOLO_FRONT_TANK_TARGET = 'lolo_front_tank_target'
