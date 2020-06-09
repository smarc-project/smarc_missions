#!/usr/bin/env python

# Christopher Iliffe Sprague
# Parent class, describing a ROS service that takes
# a PoseArray of 'coarse waypoints' and returns a Path.
# Usage: 'imc(utm) -> bt -> poseArray(map) -> path_planner -> path(map) -> bt'

import rospy, numpy as np
from scipy.interpolate import interp1d
from trajectory import Trajectory

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from nav_msgs.msg import Path
from trajectories.srv import trajectory

class Interp1d(Trajectory):

    def __init__(self):

        # become a trajectory service
        Trajectory.__init__(self, 'interp1d')

        # ROS parameters from launch file
        self.spline_degree = rospy.get_param("~spline_degree")
        self.waypoint_spacing = rospy.get_param("~waypoint_spacing")

    def interpolate(self, numpy_array):

        # independent variable: distance between points
        x = np.array([numpy_array[i+1,:] - numpy_array[i,:] for i in range(numpy_array.shape[0] - 1)])
        x = np.linalg.norm(x, axis=1)
        x = np.hstack(([0], np.cumsum(x)))

        # interpolate the data
        f = interp1d(x, numpy_array, kind=self.spline_degree, axis=0)
        x = np.arange(start=x[0], stop=x[-1], step=self.waypoint_spacing)
        y = f(x)
        return y


if __name__ == "__main__":
    traj = Interp1d()
    rospy.spin()
    # traj.test()
