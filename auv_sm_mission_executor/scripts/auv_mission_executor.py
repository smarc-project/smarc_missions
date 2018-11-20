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

import smach
import smach_ros
from smach_ros import SimpleActionState, IntrospectionServer

import rospy
import actionlib
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
import std_msgs.msg
from smarc_planning_msgs.msg import ConditionalAction, ExecutionStatus
from smarc_planning_msgs.srv import AddTask, AddTasks
import threading
from threading import Thread, Condition
from copy import deepcopy

import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy

import numpy as np
from rospy_message_converter import message_converter
import ast

class NodeState(object):
    # ROS params
    base_frame = None 
    heading_offset = None
    goal_tolerance = None
    add_task_srv = None
    add_tasks_srv = None
    listener = None

class TaskInitialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('State initialization')
        return 'succeeded'

class TaskSucceeded(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('State succeeded')
        return 'succeeded'
   
class TaskCancelled(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted'])

    def execute(self, userdata):
        rospy.loginfo('State preempted')
        return 'preempted'

class TaskFailed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['aborted'])

    def execute(self, userdata):
        rospy.loginfo('State aborted')
        return 'aborted'

class TaskPreempting(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'preempted'],
            input_keys=['task_struct'])

    def execute(self, userdata):
        rospy.loginfo('State preempting')

        userdata.task_struct[1].cancel_all_goals()
        goal_state = userdata.task_struct[1].get_state()
        while not rospy.is_shutdown() and goal_state != GoalStatus.PREEMPTED and \
              goal_state != GoalStatus.SUCCEEDED and goal_state != GoalStatus.ABORTED:
            rospy.loginfo("Waiting for server to preempt task")
            rospy.Rate(0.5).sleep()
            goal_state = userdata.task_struct[1].get_state()

        return userdata.task_struct[2]

class TaskExecution(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded', 'preempted', 'preempting', 'aborted'],
            input_keys=['task_struct'])

    def execute(self, userdata):
        rospy.loginfo('State task execution')

        # Create action client and goal dynamically. Based on mongodb
        action_tuple = self.get_task_types(userdata.task_struct[0].action_topic)
        while not rospy.is_shutdown() and len(action_tuple) == 0:
            action_tuple = self.get_task_types(userdata.task_struct[0].action_topic)
            rospy.loginfo("Waiting for action server")

        action_clz = dc_util.load_class(dc_util.type_to_class_string(action_tuple[0]))
        rospy.loginfo("Action string %s and goal string %s", action_tuple[0], action_tuple[1])

        # goal_clz = dc_util.load_class(dc_util.type_to_class_string(action_tuple[1]))
        # argument_list = self.get_arguments(userdata.task_struct[0].action_arguments)
        # mb_goal = goal_clz(*argument_list)         

        # Create action client and wait for server
        userdata.task_struct[1] = actionlib.SimpleActionClient(userdata.task_struct[0].action_topic, action_clz)
        # rospy.loginfo("Waiting for server %s with action class %s", userdata.task_struct[0].action_topic, action_clz)
        userdata.task_struct[1].wait_for_server(rospy.Duration(10))
        rospy.loginfo("Action server connected!")

        # Parse and send the goal to the action server
        print userdata.task_struct[0].action_arguments
        dictionary = ast.literal_eval(userdata.task_struct[0].action_arguments)
        print dictionary
        print action_tuple[1]
        mb_goal = message_converter.convert_dictionary_to_ros_message(action_tuple[1], dictionary)
        print mb_goal
        print mb_goal._type
        userdata.task_struct[1].send_goal(mb_goal)
        rospy.loginfo("Goal sent!")

        # Check periodically state of AUV to preempt goal when needed
        t_before = rospy.Time.now()
        rate = rospy.Rate(1)
        task_result = "running"
        while not rospy.is_shutdown() and task_result == "running":
            # Check action state from server side
            goal_state = userdata.task_struct[1].get_state()
            if goal_state == GoalStatus.REJECTED or goal_state == GoalStatus.ABORTED:
                task_result = userdata.task_struct[2] = "aborted"
                rospy.loginfo("Action aborted!")
                break
            elif self.preempt_requested(): #self.was_preempted:
                task_result = "preempting"
                userdata.task_struct[2] = "preempted"
                break
            elif goal_state == GoalStatus.PREEMPTED or self.preempt_requested(): #self.was_preempted:
                rospy.loginfo("Action preempted!")                
                task_result = userdata.task_struct[2] = "preempted"
                break
            elif goal_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Action returned success!")
                task_result = userdata.task_struct[2] = "succeeded"
                break

            # Check duration termination condition if received
            if userdata.task_struct[0].max_duration != 0.:
                t_after = rospy.Time.now()
                if t_after - t_before > userdata.task_struct[0].max_duration:
                    rospy.loginfo("Time limit reached")
                    task_result = "preempting"
                    userdata.task_struct[2] = "preempted"
                    break

            # Check waypoint termination condition if received
            if userdata.task_struct[0].x != 0.:
                if self.end_condition(userdata):
                    rospy.loginfo("Success!")
                    task_result = "preempting"
                    userdata.task_struct[2] = "succeeded"
                    break


            rate.sleep()

        rospy.loginfo("task_result %s", userdata.task_struct[2])

        # Result of executing the action 
        return task_result

    def get_task_types(self, action_name):
        result = ()
        topics = rospy.get_published_topics(action_name)
        for [topic, type] in topics:            
            if topic.endswith('feedback'):
                result = (type[:-8], type[:-14] + 'Goal')

        return result

    def end_condition(self, userdata):

        try:
            (trans, rot) = NodeState.listener.lookupTransform("/world", NodeState.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

        # Check distance to navigation goal
        start_pos = np.array(trans)
        end_pos = np.array([userdata.task_struct[0].x, userdata.task_struct[0].y, -userdata.task_struct[0].depth])
        if np.linalg.norm(start_pos - end_pos) < NodeState.goal_tolerance:
            rospy.loginfo("Reached goal!")
            return True
            
        return False

class SmachServer():
    def __init__(self):
        # State machine
        self.task_sm = None
        # List of tasks to execute
        self.tasks = []
        self.tasks_run = 0;
        self.state_lock = threading.Lock()

        # ROS params
        NodeState.base_frame = rospy.get_param('~base_frame', "lolo_auv_1/base_link")
        NodeState.heading_offset = rospy.get_param('~heading_offsets', 5.)
        NodeState.goal_tolerance = rospy.get_param('~goal_tolerance', 5.)
        NodeState.add_task_srv = rospy.get_param('~add_task_srv', '~add_task')
        NodeState.add_tasks_srv = rospy.get_param('~add_tasks_srv', '~add_tasks')
        NodeState.listener = tf.TransformListener()

        # Add states to the queue to build the sm
        rospy.Service(NodeState.add_task_srv, AddTask, self.add_task_srv_cb)
        rospy.Service(NodeState.add_tasks_srv, AddTasks, self.add_tasks_srv_cb)
        self.schedule_publisher = rospy.Publisher('current_schedule', ExecutionStatus, latch = True, queue_size = 1)

        self.update_schedule_condition = Condition()
        self.schedule_publish_thread = Thread(target=self.publish_schedule)
        self.schedule_publish_thread.start()

        # Run loop
        self.run_sm()

    def publish_schedule(self):
        """
        Loops continuous publishing the upcoming tasks to be executed.
        It is challenging to produce a list of the tasks that will be executed and when from this, so the compromises is that 
        ExecutionStatus contains the active batch with their execution_times set to now, all time-critical tasks and the next self.batch_limit normal tasks with their start time set to the end time of the current active batch.
        """
        while not rospy.is_shutdown():
            # all encompassing try/catch to make sure this loop does not go down
            try:

                # copy all relevant entries under lock 
                # we're taking a deepcopy as we might mess around with the times a bit
                with self.state_lock:
                    active_batch = deepcopy(self.tasks)

                now = rospy.get_rostime()
                # todo: fill this value better
                expected_end_of_batch = rospy.get_rostime() + rospy.Duration(120)

                # start from the time_cr
                schedule = ExecutionStatus(currently_executing = len(active_batch) > 0)

                schedule.header.stamp = now

                for m in active_batch:
                    #m.task.execution_time = now
                    schedule.execution_queue.append(m)

                self.schedule_publisher.publish(schedule)

                self.update_schedule_condition.acquire()
                self.update_schedule_condition.wait()
                self.update_schedule_condition.release()
            except Exception, e:
                rospy.logwarn('Caught exception in publish_schedule loop: %s' % e)
                rospy.sleep(1)


    def republish_schedule(self):
        """
        Notify schedule-publishing thread to update and publish schedule
        """
        self.update_schedule_condition.acquire()
        self.update_schedule_condition.notify()
        self.update_schedule_condition.release()

    def add_task_srv_cb(self, TaskReq):
        with self.state_lock:
            self.tasks.append(TaskReq.task)
        self.republish_schedule()
        return self.tasks_run
    
    def add_tasks_srv_cb(self, TasksReq):
        with self.state_lock:
            for task in TasksReq:
                self.tasks.append(task.task)
        self.republish_schedule()
        return self.tasks_run

    def monitor_cb(self, ud, msg):
        print "Got message!"
        return False

    def child_term_cb(self, outcome_map):
        if outcome_map['CONCURRENT_TASK_PREEMPTION'] == 'invalid':
            return True
        elif outcome_map['CONCURRENT_TASK_EXECUTION'] == 'succeeded':
            return True
        elif outcome_map['CONCURRENT_TASK_EXECUTION'] == 'aborted':
            return True
        elif outcome_map['CONCURRENT_TASK_EXECUTION'] == 'preempted':
            return True
        elif outcome_map['CONCURRENT_TASK_EXECUTION'] == 'preempting':
            return True
        else:
            return False

    def out_cb(self, outcome_map):
        if outcome_map['CONCURRENT_TASK_PREEMPTION'] == 'invalid':
            return 'preempting'
        else:
            return outcome_map['CONCURRENT_TASK_EXECUTION']

    def execute_task(self, task):
        rospy.loginfo("Executing task %s", self.tasks_run)

        # Reset SM 
        self.task_sm = None
        act_client = None
        task_result_flag = ""

        monitor_state = smach.Concurrence(outcomes=['succeeded', 'aborted', 'preempted', 'preempting'],
                                          default_outcome='aborted',
                                          child_termination_cb=self.child_term_cb,
                                          outcome_cb=self.out_cb)
        monitor_state.userdata.task_struct = [task, act_client, task_result_flag]

        with monitor_state:
            smach.Concurrence.add('CONCURRENT_TASK_PREEMPTION', smach_ros.MonitorState("/sm_reset", std_msgs.msg.Empty, self.monitor_cb))
            smach.Concurrence.add('CONCURRENT_TASK_EXECUTION', TaskExecution())

        # Create the state machine necessary to execute this task        
        self.task_sm = smach.StateMachine(['succeeded','aborted','preempted'])

        # Update the userdata with the new task and an action client to pass between states
        self.task_sm.userdata.task_struct = monitor_state.userdata.task_struct # [task, act_client, task_result_flag]

        with self.task_sm:

            # Initialise task data
            smach.StateMachine.add('TASK_INITIALIZATION', TaskInitialization(), transitions={'succeeded': 'TASK_EXECUTION'})
            # Final task outcomes
            smach.StateMachine.add('TASK_SUCCEEDED', TaskSucceeded(), transitions={'succeeded':'succeeded'})
            smach.StateMachine.add('TASK_CANCELLED', TaskCancelled(), transitions={'preempted':'preempted'})
            smach.StateMachine.add('TASK_FAILED', TaskFailed(), transitions={'aborted':'aborted'})
            smach.StateMachine.add('TASK_PREEMPTING', TaskPreempting(), transitions={'preempted':'TASK_CANCELLED', 'succeeded':'TASK_SUCCEEDED'})
            smach.StateMachine.add('TASK_EXECUTION', monitor_state, transitions={'preempted':'TASK_CANCELLED', 'preempting':'TASK_PREEMPTING', 'aborted':'TASK_FAILED', 'succeeded':'TASK_SUCCEEDED'})

        # Execute SM
        self.task_sm.set_initial_state(['TASK_INITIALIZATION'])
        self.task_sm.execute()

        self.republish_schedule()

    def run_sm(self):
        r = rospy.Rate(1)

        # Construct the state machine from the list of tasks
        while not rospy.is_shutdown():
            with self.state_lock:
                if len(self.tasks) == 0:
                    task_next = None
                else:
                    task_next = self.tasks.pop(0)

            if task_next is not None:
                self.execute_task(task_next)
                self.tasks_run += 1;
            else:
                # If final state reached, add IDDLE state
                rospy.loginfo("Waiting for new tasks")

            r.sleep()             

        rospy.loginfo("Killing state machine")


if __name__ == '__main__':
    rospy.init_node('smach_state_machine')
    sm_server = SmachServer()
