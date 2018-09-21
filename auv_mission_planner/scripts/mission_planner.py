#!/usr/bin/env python

import rospy
import random
import copy
import math
import os
import csv
import tf

from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))

def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def b_func(x):
    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

class MissionPlanner(object):

    def __init__(self, config_file=None):

        self._interactive = True

        self.mission_file = rospy.get_param('~mission_file', "mission.csv")
        self._server = InteractiveMarkerServer("mission_planner")

        self.waypoints = []
        self.edges = []

        self._init_menu()

        self.load_objects()

        self._server.applyChanges()

    def load_objects(self):

        if os.path.isfile(self.mission_file):
            with open(self.mission_file) as csvfile:
                spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
                for row in spamreader:
                    rospy.loginfo("Got entry: %s", " ".join(row))
                    pose = Pose()
                    pose.position.x = float(row[3])
                    pose.position.y = float(row[4])
                    pose.position.z = -float(row[2])
                    self.waypoints.append(pose)

        if len(self.waypoints) == 0:
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = -85.

            self.waypoints.append(pose)

        # Draw the ROI
        self.draw_waypoints()

    ## Initialize the right-click menu
    def _init_menu(self):

        self.menu_handler = MenuHandler()

        add_point_entry = self.menu_handler.insert( "Add Waypoint", callback=self._add_point_cb)
        del_point_entry = self.menu_handler.insert( "Delete Waypoint", callback=self._del_point_cb)
        save_plan_entry = self.menu_handler.insert( "Save mission plan", callback=self._save_plan_cb)

        enable_entry = self.menu_handler.insert( "Movement control", callback=self._enable_cb )
        self.menu_handler.setCheckState( enable_entry, MenuHandler.CHECKED )

    # Add Vertex callback
    def _save_plan_cb(self, feedback):

        #This is the object that we are pressing (feedback) so
        #that we can get the marker name etc..
        rospy.loginfo("Saving the plan to file: %s", self.mission_file)

        with open(self.mission_file, 'w') as csvfile:
            #uint64 task_id, float64 altitude, float64 depth, float64 x, float64 y, float64 theta, string action_topic, duration max_duration, smarc_msgs/StringArray[] action_arguments

            thetas = []
            for i in range(0, len(self.waypoints)-1):
                xdiff = (self.waypoints[i+1].position.x - self.waypoints[i].position.x)
                ydiff = (self.waypoints[i+1].position.y - self.waypoints[i].position.y)
                thetas.append(180./math.pi*math.atan2(ydiff, xdiff))
            if len(thetas) == 1:
                thetas.append(0.)
            else:
                thetas.append(thetas[-1])

            spamwriter = csv.writer(csvfile, delimiter=' ', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for waypoint_index, pose in enumerate(self.waypoints):
                theta = thetas[waypoint_index]
                quaternion = tf.transformations.quaternion_from_euler(0., 0., math.pi/180.*theta)
                depth = -pose.position.z
                duration = 100.
                arguments = "[%s, %s, %f, %f, %f, %f, %f, %f, %f]" % ("____pose____", "world", pose.position.x, pose.position.y, -depth, quaternion[0], quaternion[1], quaternion[2], quaternion[3])
                print arguments
                spamwriter.writerow([waypoint_index, 0, depth, pose.position.x, pose.position.y, theta, "/bezier_planner", duration, arguments])


    # Add Vertex callback
    def _add_point_cb(self, feedback):

        #This is the object that we are pressing (feedback) so
        #that we can get the marker name etc..
        rospy.loginfo("Add point from marker: %s", feedback.marker_name)

        # Get the pose and create the new object a little away
        pose = feedback.pose
        pose.position.x = pose.position.x+1.0*math.cos(math.radians(90))
        pose.position.y = pose.position.y+1.0*math.cos(math.radians(45))
        ######################################################

        # Add object
        waypoint_index = int(feedback.marker_name.split('_')[1]) + 1
        self.waypoints[waypoint_index:waypoint_index] = [pose]

        # Draw the ROI
        self.draw_waypoints()

    # Delete Vertex callback
    def _del_point_cb(self, feedback):

        rospy.loginfo("Delete point: %s", feedback.marker_name)

        waypoint_index = int(feedback.marker_name.split('_')[1])

        if len(self.waypoints) <= 1:
            rospy.logerr("The minimum number of waypoints is 1!")
            return

        rospy.loginfo("Deleting waypoint %d: out of: %d", waypoint_index, len(self.waypoints))
        # We only want to delete particular marker
        del self.waypoints[waypoint_index]

        self._server.erase("Waypoint_" + str(len(self.waypoints)))
        self.draw_waypoints()

    def _update_poly(self, feedback):

        if feedback.control_name.startswith("move_plane") or \
           feedback.control_name.startswith("move_axis"):
            waypoint_index = int(feedback.marker_name.split('_')[1])
            print "Setting new pose for waypoint: ", waypoint_index
            self.waypoints[waypoint_index] = feedback.pose
            int_marker = self.create_line_marker()
            self._server.erase("Line")
            self._server.insert(int_marker)
            self._server.applyChanges()

    def _enable_cb(self, feedback):

        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if state == MenuHandler.CHECKED:
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self._interactive = False
        else:
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self._interactive = True

        self.menu_handler.reApply( self._server )

        self.draw_waypoints()

    def draw_waypoints(self):

        for current_index, pose in enumerate(self.waypoints):
            rospy.loginfo("Inserting waypoint: %s", "Waypoint_" + str(current_index))
            int_marker = self.create_waypoint_marker(pose, current_index)
            self._server.erase("Waypoint_" + str(current_index))
            self._server.applyChanges()
            self._server.insert(int_marker, self._update_poly)
            self.menu_handler.apply(self._server, "Waypoint_" + str(current_index))
            self._server.applyChanges()

        int_marker = self.create_line_marker()
        self._server.erase("Line")
        self._server.insert(int_marker, self._update_poly)
        self._server.applyChanges()

    # This part draws the line strips between the points
    def create_line_marker(self):

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = "Line"
        int_marker.description = ""
        int_marker.pose = self.waypoints[0]

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1

        #random.seed()
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( marker )

        int_marker.controls.append(control)

        marker.points = []
        for wp_pose in self.waypoints:
            p = Point()
            p.x = wp_pose.position.x - int_marker.pose.position.x
            p.y = wp_pose.position.y - int_marker.pose.position.y
            p.z = wp_pose.position.z - int_marker.pose.position.z
            marker.points.append(p)

        return int_marker

    def create_waypoint_marker(self, pose, current_index):

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = "Waypoint_" + str(current_index)
        int_marker.description = "Waypoint " + str(current_index)
        int_marker.pose = pose
        #int_marker.pose.position.z = 0.01

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        #int_marker.pose.position.z = (marker.scale.z / 2)

        #random.seed(soma_type)
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0

        #marker.pose = pose
        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.name = "move_plane"

        if self._interactive:
            int_marker.controls.append(copy.deepcopy(control))
        
        control.name = "move_axis"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        if self._interactive:
            int_marker.controls.append(copy.deepcopy(control))

        # add menu control
        menu_control = InteractiveMarkerControl()

        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True

        menu_control.markers.append( marker) #makeBox(int_marker) )
        int_marker.controls.append(menu_control)

        return int_marker


if __name__ == "__main__":

    rospy.init_node('mission_planner', anonymous=True)
    mission_planner = MissionPlanner()
    rospy.spin()
