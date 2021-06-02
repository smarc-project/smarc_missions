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
                               config.MIN_ALTITUDE, -1, 50)
        self.ddrc.add_variable(bb_enums.MAX_DEPTH,
                               "default:{}".format(config.MAX_DEPTH),
                               config.MAX_DEPTH, 0, 50)

        self.ddrc.add_variable(bb_enums.WAYPOINT_TOLERANCE,
                               "default:{}".format(config.WAYPOINT_TOLERANCE),
                               config.WAYPOINT_TOLERANCE, 0.1, 10)

        # coverage stuffs
        self.ddrc.add_variable(bb_enums.SWATH,
                               "default:{}".format(config.SWATH),
                               config.SWATH, 1, 100)
        self.ddrc.add_variable(bb_enums.LOCALIZATION_ERROR_GROWTH,
                               "default:{}".format(config.LOCALIZATION_ERROR_GROWTH),
                               config.LOCALIZATION_ERROR_GROWTH, 0, 0.1)

        self.ddrc.add_variable(bb_enums.MISSION_LOG_FOLDER,
                               "default:{}".format(config.MISSION_LOG_FOLDER),
                               config.MISSION_LOG_FOLDER)

        self.ddrc.add_variable(bb_enums.ENABLE_MANUAL_MISSION_LOG,
                               "Check to start a new log, uncheck to stop and save it",
                               config.ENABLE_MANUAL_MISSION_LOG)


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
