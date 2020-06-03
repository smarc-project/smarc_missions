#!/usr/bin/env python3

# Christopher Iliffe Sprague
# Parent class, describing a ROS service that takes
# a PoseArray of 'coarse waypoints' and returns a Path.
# Usage: 'imc(utm) -> bt -> poseArray(map) -> path_planner -> path(map) -> bt'

import rospy, numpy as np
from scipy.interpolate import splprep, splev
from trajectory import Trajectory

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from nav_msgs.msg import Path
from trajectories.srv import trajectory

class BSpline(Trajectory):

    def __init__(self):

        # become a trajectory service
        Trajectory.__init__(self, 'bspline')

        # ROS parameters from launch file
        self.spline_degree = rospy.get_param("~spline_degree")
        self.smoothing_factor = rospy.get_param("~smoothing_factor")
        self.waypoint_spacing = rospy.get_param("~waypoint_spacing")

    def interpolate(self, numpy_array):

        # independent variable: distance between points
        x = np.array([numpy_array[i+1,:] - numpy_array[i,:] for i in range(numpy_array.shape[0] - 1)])
        x = np.linalg.norm(x, axis=1)
        x = np.hstack(([0], np.cumsum(x)))

        # interpolate
        tck, u = splprep(
            [*numpy_array.transpose()], 
            u=x,
            k=self.spline_degree,
            s=self.smoothing_factor
        )
        
        # sample
        u = np.arange(start=x[0], stop=x[-1], step=self.waypoint_spacing)
        x, y, z = splev(u, tck)
        samples = np.vstack((x, y, z)).transpose()
        return samples



if __name__ == "__main__":
    traj = BSpline()
    rospy.spin()
    # traj.test()