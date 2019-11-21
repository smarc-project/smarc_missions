#!/usr/bin/env python

# Copyright 2018 Nils Bore (nbore@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import actionlib
from smarc_planning_msgs.msg import ConditionalAction
from smarc_planning_msgs.srv import AddTask, AddTasks, AddTaskRequest, AddTaskResponse
from smarc_planning_msgs import smarc_task_utils
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

            master_task = ConditionalAction(action_topic=row[7])
            master_task.task_id = int(row[0])
            master_task.x = float(row[1])
            master_task.y = float(row[2])
            master_task.depth = float(row[3])
            master_task.altitude = float(row[4])
            master_task.theta = float(row[5])
            smarc_task_utils.add_duration_argument(master_task, float(row[6]))

            if len(row) > 8 and len(row[8]) > 2:
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
