#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)


from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
import py_trees as pt
import bb_enums
import rospy


class ReconfigServer(object):
    def __init__(self, config):
        """
        Read the defaults from the config object
        so that we dont change the launch-defaults right away
        with different reconfig defaults

        We then just put whatever reconfig we get into the BB with the same key
        """

        self.bb = pt.blackboard.Blackboard()

        # DynamicDynamicReConfig
        # because the .cfg way of doing this is pain
        self.ddrc = DDynamicReconfigure("smarc_bt_reconfig")
        # name, description, default value, min, max, edit_method
        self.ddrc.add_variable(bb_enums.MIN_ALTITUDE,
                               "default:{}".format(config.MIN_ALTITUDE),
                               float(config.MIN_ALTITUDE), -1, 50)

        self.ddrc.add_variable(bb_enums.MAX_DEPTH,
                               "default:{}".format(config.MAX_DEPTH),
                               float(config.MAX_DEPTH), 0, 50)


        # coverage stuffs
        self.ddrc.add_variable(bb_enums.SWATH,
                               "default:{}".format(config.SWATH),
                               float(config.SWATH), 1, 100)
        self.ddrc.add_variable(bb_enums.LOCALIZATION_ERROR_GROWTH,
                               "default:{}".format(config.LOCALIZATION_ERROR_GROWTH),
                               float(config.LOCALIZATION_ERROR_GROWTH), 0, 0.1)


        # dubins planner stuff
        self.ddrc.add_variable(bb_enums.DUBINS_TURNING_RADIUS,
                               "default:{}".format(config.DUBINS_TURNING_RADIUS),
                               float(config.DUBINS_TURNING_RADIUS), 0.1, 50)

        self.ddrc.add_variable(bb_enums.DUBINS_INTERSECTION_RADIUS,
                               "Only used if the dubins path is set to cut the corner. Radius of the circle centered on the original waypoint itself, used to compute the intersection waypoints with the straight path. default:{}".format(
                                   config.DUBINS_INTERSECTION_RADIUS),
                               float(config.DUBINS_INTERSECTION_RADIUS), 0.1, 100)

        self.ddrc.add_variable(bb_enums.DUBINS_COMPUTE_PATH,
                               "Check to compute the dubins path between waypoints, uncheck to use the basic planner",
                               config.DUBINS_COMPUTE_PATH)


        rospy.loginfo("Started dynamic reconfig server with keys:{}".format(self.ddrc.get_variable_names()))


        # this should be the last thing in this init
        self.ddrc.start(self.reconfig_cb)



    def reconfig_cb(self, config, level):
        for key in self.ddrc.get_variable_names():
            new_value = config.get(key)
            old_value = self.bb.get(key)

            if old_value is None or old_value != new_value:
                rospy.loginfo("New value for:{} set to:{} (was {})".format(key, new_value, old_value))
                self.bb.set(key, new_value)

        return config
