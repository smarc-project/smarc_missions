#! /usr/bin/env python

# standard ros server things
import rospy, actionlib, py_trees
import std_msgs.msg as std_msgs

import actionlib
import dynamic_reconfigure.server
import rospy
import py_trees_ros
#from sam_march.msg import GenericStringAction

from py_trees_ros.mock.action_server import ActionServer
from sam_march.msg import GenericStringAction


class Emergency(ActionServer):

    def __init__(self):
        ActionServer.__init__(self, '/sam_emergency', GenericStringAction, self.worker)

    def worker(self):
        print("yolo")


if __name__ == "__main__":

    # emergency action server
    rospy.init_node('emergency_action')
    act = Emergency()
    act.start()
    rospy.spin()
