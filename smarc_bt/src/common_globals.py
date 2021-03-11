#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


########################
# DEFAULT VALUES
########################
# Hz.
BT_TICK_RATE = 3

# these are from croatia, biograd coast
DEFAULT_UTM_ZONE = 33
DEFAULT_UTM_BAND = 'T'
# if set to true, utm zone and band will be set from
# the gps fix we read instead of rosparams
TRUST_GPS = False # True

CBF_BT_TOPIC = 'cbf_bt/active_limits'
# to ensure that the condition list is reset properly, set to True.
CHECK_CBF_LIST = True

# how close do we expect the path planned waypoints to be to the coarse
# plans a user creates
COARSE_PLAN_REFINED_PLAN_THRESHOLD = 1

# if a poi is Xm away from the latest, its a new one
POI_DIST = 10

SETUP_TIMEOUT = 1.0



