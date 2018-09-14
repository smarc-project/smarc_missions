#!/usr/bin/env python

import smach
import smach_ros
from smach_ros import SimpleActionState, IntrospectionServer

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from smarc_msgs.msg import SMTask
from smarc_msgs.srv import AddTask
import threading

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state INIT')
        if self.counter < 3:
            return 'outcome1'
        else:
            return 'outcome2'

class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state END')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

class ErrorState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ERROR')
        return 'outcome1'

# State for calling action servers
class CallActState(smach.State):
    def __init__(self, act_name):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state CallActState')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


class SmachServer():
    def __init__(self):
        self.tasks = Queue()
        self.tasks_run = 0;
        
        # Create a SMACH state machine
        self.task_sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

        # Add error state
        with self.task_sm:
            smach.StateMachine.add('ERROR', ErrorState(), transitions={'outcome1':'aborted'})

        # Add states to the queue to build the sm
        rospy.Service("/task_executor/" + "add_state", AddTask, self.add_state_srv)
        
        # Run loop
        self.run_sm()

    def add_state_srv(self, TaskReq):
        self.tasks.put(TaskReq.task)

    def run_sm(self):
        # Create INIT state and add to buffer
        init_task = SMTask()
        init_task.state   = "INIT"
        init_task.task_id = 1
        self.tasks_run = 1
        task_buff = init_task

        # Construct the state machine from the list of tasks
        r = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            self.tasks_run += 1;
            try:
                task_current = task_buff
                task_next = self.tasks.get(False)
                task_next.state = "ACTION"
            except Empty, e:
                # If final state reached, add IDDLE state
                iddle_task = SMTask()
                iddle_task.state   = "IDLE"
                task_next = iddle_task

            task_next.task_id = self.tasks_run
            self.add_task_sm(task_current, task_next)
            task_buff = task_next

            r.sleep()

            # Execute state machine when all states from the queue are set
            if self.tasks.empty():
                self.task_sm.execute()

    def goal_callback(userdata, default_goal):
        mb_goal = MoveBaseGoal()
        goal.goal = 2
        return goal

    def add_task_sm(self, task_current, task_next):

        with self.task_sm:
            if task_current.state == "ACTION":
                action_name = ""    # TODO
                task_cb = smach_ros.SimpleActionState(action_name, MoveBaseAction, MoveBaseGoal())
            else if task_current.state == "IDLE":
                task_cb = IdleState()
            else if task_current.state == "INIT":
                task_cb = InitState()
            else: 
                # Error handling 
                task_cb = ErrorState()
                task_current.task_id = 666
                task_current.task_id = 667

            # Create new state
            smach.StateMachine.add('state_' + task_current.task_id, task_cb, 
                transitions={'outcome1': task_next.state, 'outcome2':'ERROR'})


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')
    sm_server = SmachServer()
