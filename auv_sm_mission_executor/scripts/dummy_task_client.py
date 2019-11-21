#!/usr/bin/env python

# Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
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
