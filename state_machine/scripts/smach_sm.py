#!/usr/bin/env python

import smach
import smach_ros
from smach_ros import SimpleActionState, IntrospectionServer

import rospy
import actionlib
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from smarc_msgs.msg import SMTask
from smarc_msgs.srv import AddTask
import threading

import numpy as np

class NodeState(object):
    # ROS params
    base_frame = None 
    heading_offset = None
    goal_tolerance = None
    add_task_srv = None
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


class TaskExecution(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['task'])

    def execute(self, userdata):
        rospy.loginfo('State task execution')
        # Create action client and wait for server
        self.act_client = actionlib.SimpleActionClient(userdata.task.action_topic, MoveBaseAction)
        self.act_client.wait_for_server(rospy.Duration(10))

        rospy.loginfo("Action server connected!")

        # Create new action goal
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = "/world"
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.pose.position.x = userdata.task.x
        mb_goal.target_pose.pose.position.y = userdata.task.y
        mb_goal.target_pose.pose.position.z = userdata.task.depth
        mb_goal.target_pose.pose.orientation.x = 0.0    # TODO: add yaw
        mb_goal.target_pose.pose.orientation.y = 0.0
        mb_goal.target_pose.pose.orientation.z = 0.0
        mb_goal.target_pose.pose.orientation.w = 1.0

        # Sends the goal to the action server.
        self.act_client.send_goal(mb_goal)

        # Check periodically state of AUV to preempt goal when needed
        t_before = rospy.Time.now()
        rate = rospy.Rate(1)
        task_result = "running"
        while not rospy.is_shutdown() and task_result == "running":
            # Check pose of AUV  and compare to goal for terminating condition
            task_result = self.end_condition(mb_goal)
            if task_result == "succeeded":
                rospy.loginfo("Success!")
                break

            # Check state from server side
            goal_state = self.act_client.get_state()
            if goal_state == GoalStatus.REJECTED or goal_state == GoalStatus.ABORTED:
                task_result = "aborted"
                rospy.loginfo("Action aborted!")
            elif goal_state == GoalStatus.PREEMPTED:
                rospy.loginfo("Action preempted!")                
                task_result = "preempted"

            t_after = rospy.Time.now()
            if t_after - t_before > userdata.task.max_duration:
                rospy.loginfo("Time limit reached")                                
                task_result = "preempted"
                break

            rate.sleep()

        # Result of executing the action 
        return task_result

    def end_condition(self, mb_goal):

        goal_state = "running"

        try:
            (trans, rot) = NodeState.listener.lookupTransform("/world", NodeState.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return goal_state

        # Check distance to navigation goal
        start_pos = np.array(trans)
        end_pos = np.array([mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y, mb_goal.target_pose.pose.position.z])
        if np.linalg.norm(start_pos - end_pos) < NodeState.goal_tolerance:
            rospy.loginfo("Reached goal!")
            goal_state = "succeeded"
            
        return goal_state

class SmachServer():
    def __init__(self):
        self.task_sm = None
        self.tasks = []
        self.tasks_run = 0;

        # ROS params
        NodeState.base_frame = rospy.get_param('~base_frame', "lolo_auv_1/base_link")
        NodeState.heading_offset = rospy.get_param('~heading_offsets', 5.)
        NodeState.goal_tolerance = rospy.get_param('~goal_tolerance', 5.)
        NodeState.add_task_srv = rospy.get_param('~add_task_srv', 5.)
        NodeState.listener = tf.TransformListener()

        # Add states to the queue to build the sm
        rospy.Service(NodeState.add_task_srv, AddTask, self.add_state_srv)

        # Run loop
        self.run_sm()

    def reset_sm(self):
        self.task_sm = None

    def add_state_srv(self, TaskReq):
        self.tasks.append(TaskReq.task)

    def execute_task(self, task):
        rospy.loginfo('Execution of task %s was requested' % task.task_id)

        # Reset SM object
        self.reset_sm()

        # Create the state machine necessary to execute this task        
        self.task_sm = smach.StateMachine(['succeeded','aborted','preempted'])

        # Update the userdata with the new task
        self.task_sm.userdata.task = task

        with self.task_sm:

            # Initialise task data
            smach.StateMachine.add('TASK_INITIALIZATION', TaskInitialization(), transitions={'succeeded': 'TASK_EXECUTION'})
            
            # Final task outcomes
            smach.StateMachine.add('TASK_SUCCEEDED', TaskSucceeded(), transitions={'succeeded':'succeeded'})
            smach.StateMachine.add('TASK_CANCELLED', TaskCancelled(), transitions={'preempted':'preempted'})
            smach.StateMachine.add('TASK_FAILED', TaskFailed(), transitions={'aborted':'aborted'})
            smach.StateMachine.add('TASK_EXECUTION', TaskExecution(), 
                transitions={'preempted':'TASK_CANCELLED', 'aborted':'TASK_FAILED', 'succeeded':'TASK_SUCCEEDED'})

        # Execute SM
        self.task_sm.set_initial_state(['TASK_INITIALIZATION'])
        self.task_sm.execute()

    def run_sm(self):
        r = rospy.Rate(1)

        # Construct the state machine from the list of tasks
        while not rospy.is_shutdown():
            if not len(self.tasks) == 0:
                task_next = self.tasks.pop(0)
                rospy.loginfo("Executing task %s", self.tasks_run)
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
