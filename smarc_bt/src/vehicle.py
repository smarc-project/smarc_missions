#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

import time

import rospy, tf
from geometry_msgs.msg import PointStamped
from geographic_msgs.msg import GeoPoint
from smarc_msgs.msg import DVL, Leak, ThrusterFeedback
from smarc_bt.msg import GotoWaypoint
from sensor_msgs.msg import NavSatFix, BatteryState
from sam_msgs.msg import PercentStamped


class StringAnimation(object):
    """
    A nice little animation thing to show updates happening
    in a string format
    """
    def __init__(self, num_slots):
        self._frames = "◜◝◞◟"
        self._current_frames = [-1] * num_slots
        self._str = "X" * num_slots

    def update(self, slot):
        self._current_frames[slot] = (self._current_frames[slot] + 1) % len(self._frames)

    def __str__(self):
        s = ""
        for slot in range(len(self._str)):
            if self._current_frames[slot] == -1:
                f = 'X'
            else:
                f = self._frames[self._current_frames[slot]]
            s += f
        self._str = s
        return self._str



class Vehicle(object):
    """
    A common vehicle object to keep track of all common
    data about a vehicle
    """
    def __init__(self,
                 auv_config):

        self.auv_config = auv_config
        self.robot_name = auv_config.robot_name

        # for visualizations
        self._animation = StringAnimation(num_slots=5)
        self.last_goto_wp = GotoWaypoint()
        self._last_wp_pub = rospy.Publisher(self.auv_config.LAST_WP_TOPIC, GotoWaypoint, queue_size=1)

        self._init_tf_vars()
        # some state strings to be reported in case of trouble
        self._status_str_tf = "Uninitialized"
        self._last_update_tf = -1

        # these will come from dvl
        self.altitude = None
        self.dvl_velocity_msg = None
        self._dvl_sub = rospy.Subscriber(self.auv_config.DVL_TOPIC, DVL, self._dvl_cb, queue_size=2)
        self._status_str_dvl = "Uninitialized"
        self._last_update_dvl = -1

        # leak...
        self.leak = None
        self._leak_sub = rospy.Subscriber(self.auv_config.LEAK_TOPIC, Leak, self._leak_cb, queue_size=2)
        self._status_str_leak = "Uninitialized"
        self._last_update_leak = -1

        # raw lat lon 
        self.position_latlon = [None, None]
        self._latlon_sub = rospy.Subscriber(self.auv_config.LATLON_TOPIC, GeoPoint, self._latlon_cb, queue_size=2)

        # raw GPS object
        self.raw_gps_obj = None
        self._gps_sub = rospy.Subscriber(self.auv_config.GPS_TOPIC, NavSatFix, self._gps_cb, queue_size=2)
        self._status_str_gps = "Uninitialized"
        self._last_update_gps = -1

        # VBS, LCG
        self.vbs = None
        self.lcg = None
        self._vbs_sub = rospy.Subscriber(self.auv_config.VBS_TOPIC, PercentStamped, self._vbs_cb, queue_size=2)
        self._lcg_sub = rospy.Subscriber(self.auv_config.LCG_TOPIC, PercentStamped, self._lcg_cb, queue_size=2)

        # thrusters
        self.t1 = None
        self.t2 = None
        self._t1_sub = rospy.Subscriber(self.auv_config.T1_TOPIC, ThrusterFeedback, self._t1_cb, queue_size=2)
        self._t2_sub = rospy.Subscriber(self.auv_config.T2_TOPIC, ThrusterFeedback, self._t2_cb, queue_size=2)

        # battery
        self.batt_v = None
        self.batt_percent = None
        self._batt_sub = rospy.Subscriber(self.auv_config.BATT_TOPIC, BatteryState, self._batt_cb, queue_size=2)



    def __str__(self):
        anim = self._animation.__str__()
        status = [
            ('TF', anim[0], 'Depth:{}'.format(self.depth)),
            ('DVL', anim[1], 'Alt:{}'.format(self.altitude)),
            ('Leak', anim[2], self._status_str_leak),
            ('Latlon', anim[3], str(self.position_latlon)),
            ('GPS', anim[4], self._status_str_gps)
        ]

        s = ""

        for name, frame, string in status:
            s += "{}:{} - {}\n".format(name, frame, string)
        return s


    def _init_tf_vars(self):
        # these will come from the TF tree
        # and are None'd at every update attempt
        # position does not include height or depth to avoid confusion
        # use depth for that
        self.position_utm = [None, None]
        self.orientation_quat = [None, None, None, None]
        self.orientation_rpy = [None, None, None]
        self.depth = None
        # for convenicent use in ROS elsewhere
        self.position_point_stamped = None


    def setup_tf_listener(self, timeout_secs=120):
        """
        create a tf listener to be used later and return it
        because we cant store a tf listener in the blackboard of a BT
        due to serialization problems
        so we just... dont store it in this object...
        """
        listener = tf.TransformListener()
        try:
            listener.waitForTransform(self.auv_config.UTM_LINK,
                                      self.auv_config.BASE_LINK,
                                      rospy.Time(),
                                      rospy.Duration(secs=timeout_secs))
            self._status_str_tf = "Got xform"
            return listener
        except:
            self._status_str_tf = "waitForTransform failed from '{}' to '{}' after {}s, is the TF tree in one piece?".format(self.auv_config.UTM_LINK, self.auv_config.BASE_LINK, timeout_secs)
            return None


    def tick(self, tf_listener):
        """
        mimic the behaviour of the BT, since this should be in lock-step with it
        """
        self._update_tf(tf_listener)
        self._last_wp_pub.publish(self.last_goto_wp)


    def _update_tf(self, listener):
        # init the vars so that we can catch later if they are
        # updated properly
        self._init_tf_vars()
        # create a fresh  listener every time, because the BB will break
        # when we put this object in it with the listener as a variable
        try:
            posi, ori = listener.lookupTransform(self.auv_config.UTM_LINK,
                                                 self.auv_config.BASE_LINK,
                                                 rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            self._status_str_tf = "lookupTransform failed from '{}' to '{}', is the TF tree in one piece?".format(self.auv_config.UTM_LINK, self.auv_config.BASE_LINK)
            return
        except Exception as e:
            self._status_str_tf = "lookupTransform failed:\n{}".format(e)
            return


        # position for x,y
        self.position_utm = [posi[0], posi[1]]
        # depth for z.
        self.depth = -posi[2]
        self.orientation_quat = [ori[0], ori[1], ori[2], ori[3]]
        rpy = tf.transformations.euler_from_quaternion(ori)
        self.orientation_rpy = [rpy[0], rpy[1], rpy[2]]

        ps = PointStamped()
        ps.header.frame_id = self.auv_config.UTM_LINK
        ps.header.stamp = rospy.Time(0)
        ps.point.x = posi[0]
        ps.point.y = posi[1]
        ps.point.z = posi[2]
        self.position_point_stamped = ps

        self._status_str_tf = "TF Up to date"
        self._last_update_tf = time.time()
        self._animation.update(0)


    def _dvl_cb(self, msg):
        self.altitude = msg.altitude
        self.dvl_velocity_msg = msg.velocity
        self._last_update_dvl = time.time()
        self._status_str_dvl = "Working"
        self._animation.update(1)

    def _leak_cb(self, msg):
        self.leak = msg.value
        self._last_update_leak = time.time()
        self._status_str_leak = "Working"
        self._animation.update(2)

    def _latlon_cb(self, msg):
        self.position_latlon = [msg.latitude, msg.longitude]
        self._animation.update(3)

    def _gps_cb(self, msg):
        self.raw_gps_obj = msg
        self._status_str_gps = "Working"
        self._last_update_gps = time.time()
        self._animation.update(4)

    def _vbs_cb(self, msg):
        self.vbs = msg.value

    def _lcg_cb(self, msg):
        self.lcg = msg.value

    def _t1_cb(self, msg):
        self.t1 = msg.rpm.rpm

    def _t2_cb(self, msg):
        self.t2 = msg.rpm.rpm

    def _batt_cb(self, msg):
        self.batt_v = msg.voltage
        self.batt_percent = msg.percentage

