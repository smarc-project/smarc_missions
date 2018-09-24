#!/usr/bin/env python

import rospy
import actionlib
from smarc_msgs.msg import SMTask, StringArray
from smarc_msgs.srv import AddTask, AddTasks, AddTaskRequest, AddTaskResponse
from smarc_msgs import sm_task_utils
from geometry_msgs.msg import PoseStamped
import csv
import sys
import os

def mission_tasks(mission_file):
    """ 
    Create an example of a task which we'll copy for other tasks later.
    This is a good example of creating a task with a variety of arguments.
    """


    if not os.path.isfile(mission_file):
        rospy.logerr("No such file: %s", mission_file)
        sys.exit(-1)

    tasks = []
    with open(mission_file) as csvfile:
        spamreader = csv.reader(csvfile, delimiter=' ', quotechar='"')
        for row in spamreader:
            rospy.loginfo("Got entry: %s", " ".join(row))

            master_task = SMTask(action_topic=row[7])
            #sm_task_utils.add_pose_stamped_argument(master_task, pose_stamped)
            master_task.task_id = int(row[0])
            master_task.x = float(row[1])
            master_task.y = float(row[2])
            master_task.depth = float(row[3])
            master_task.altitude = float(row[4])
            master_task.theta = float(row[5])
            sm_task_utils.add_duration_argument(master_task, float(row[6]))

            if len(row[8]) > 2:
                master_task.action_arguments = row[8]
                print master_task.action_arguments
            else:
                master_task.action_arguments = "{}"

            tasks.append(master_task)
            print master_task

    return tasks

if __name__ == '__main__':

    rospy.init_node("task_routine", log_level=rospy.INFO)
 
    mission_file = rospy.get_param('~mission_file', "mission.csv")
    rospy.wait_for_service('/task_executor/add_state')

    tasks = mission_tasks(mission_file)
    for task in tasks:
        try:
            add_task = rospy.ServiceProxy('/task_executor/add_state', AddTask)
            add_task_res = add_task(task)        
            print "response %s" %add_task_res.task_id
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    rospy.loginfo("Successfully added %d tasks, exiting...", len(tasks))
