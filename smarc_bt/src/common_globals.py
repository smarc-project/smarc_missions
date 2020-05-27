#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


########################
# DEFAULT VALUES
########################
# in ms.
BT_TICKING_PERIOD = 50

# these are from croatia, biograd coast
DEFAULT_UTM_ZONE = 33
DEFAULT_UTM_BAND = 'T'
# if set to true, utm zone and band will be set from
# the gps fix we read instead of rosparams
TRUST_GPS = True

CBF_BT_TOPIC = 'cbf_bt/active_limits'
# to ensure that the condition list is reset properly, set to True.
CHECK_CBF_LIST = True
