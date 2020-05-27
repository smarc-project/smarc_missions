#!/usr/bin/env python3

# Christopher Iliffe Sprague
# Parent class, describing a ROS service that takes
# a PoseArray of 'coarse waypoints' and returns a Path.
# Usage: 'imc(utm) -> bt -> poseArray(map) -> path_planner -> path(map) -> bt'

import rospy, numpy as np
from scipy.interpolate import interp1d

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from nav_msgs.msg import Path
from trajectories.srv import trajectory

class Interp1d:

    def __init__(self):

        # start node
        rospy.init_node('interp1d')
        rospy.Service('interp1d', trajectory, self.__call__)

        # ROS parameters from launch file
        self.spline_degree = rospy.get_param("~spline_degree")
        self.n_points = rospy.get_param("~n_points")

    def __call__(self, pose_array):
        
        # sanity
        assert(isinstance(pose_array, PoseArray))

        # convert PoseArray to numpy
        y = np.array([[
            pose.position.x, 
            pose.position.y, 
            pose.position.z
        ] for pose in pose_array.poses])

        # interpolate those points
        y = self.compute(y)

        # convert to Path
        path = Path()
        for i in range(y.shape[0]):
            pose_stamped = PoseStamped()
            pose = Pose()
            point = Point()
            point.x, point.y, point.z = y[i,:]
            pose.position = point
            pose_stamped.pose = pose
            path.poses.append(pose_stamped)
        return path

    def compute(self, numpy_array):

        # assume uniform spacing (e.g. temporally)
        x = np.linspace(0, 1, num=numpy_array.shape[0])

        # interpolate the data
        f = interp1d(x, numpy_array, kind=self.spline_degree, axis=0)
        x = np.linspace(0, 1, num=self.n_points)
        y = f(x)
        return y

def test():

    # initialise plot
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1)

    # make random 3D waypoints (but, only plot 3D)
    n = 20
    y = np.random.uniform(low=-1, high=1, size=(n,3))
    ax.plot(y[:,0], y[:,1], 'k.-')

    # interpolate
    y = Interp1d().compute(y)
    ax.plot(y[:,0], y[:,1], 'k--')

    # save the figure
    fig.savefig('example_interp1d.png', bbox_inches='tight')




if __name__ == "__main__":
    Interp1d()
    rospy.spin()
    # test()