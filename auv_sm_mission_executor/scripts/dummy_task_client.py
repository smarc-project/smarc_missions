#!/usr/bin/env python

import rospy
import actionlib
from smarc_planning_msgs.msg import ConditionalAction
from smarc_planning_msgs.srv import AddTask, AddTaskRequest, AddTaskResponse
from smarc_planning_msgs import smarc_task_utils
from geometry_msgs.msg import PoseStamped
        

def dummy_task():
    """ 
    Create an example of a task which we'll copy for other tasks later.
    This is a good example of creating a task with a variety of arguments.
    """

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "/world"
    pose_stamped.pose.position.x = 200.0
    pose_stamped.pose.position.y = 1.0
    pose_stamped.pose.position.z = -20.0
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    master_task = ConditionalAction(action_topic='/bezier_planner')        
    #sm_task_utils.add_pose_stamped_argument(master_task, pose_stamped)
    smarc_task_utils.add_duration_argument(master_task, 10)
    
    return master_task

if __name__ == '__main__':

    rospy.init_node("task_routine", log_level=rospy.INFO)
 
    rospy.wait_for_service('/task_executor/add_state')

    r = rospy.Rate(5)
    # wait for simulated time to kick in as rospy.get_rostime() is 0 until first clock message received
    while not rospy.is_shutdown():
        task = dummy_task()
        try:
            add_task = rospy.ServiceProxy('/task_executor/add_state', AddTask)
            add_task_res = add_task(task)        
            print "response %s" %add_task_res.task_id
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        r.sleep()
