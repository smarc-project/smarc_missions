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
            outcomes=['succeeded, preempted, aborted'],
            input_keys=['task'])

    def execute(self, userdata):
        rospy.loginfo('State task execution')
        # Create action client and wait for server
        self.act_client = actionlib.SimpleActionClient(userdata.task.action, MoveBaseAction)
        self.assert(self.act_client.wait_for_server(rospy.Duration(10)))

        # Create new action goal
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = "world"
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
        while not rospy.is_shutdown() and abs(t_after - t_before) < userdata.task.max_duration and task_result == "running":
            # TODO: check pose of AUV from topics and compare to goal manually
                        
            # Check state from server side
            goal_state = self.act_client.get_state()
            if goal_state == GoalStatus.REJECTED || goal_state == GoalStatus.ABORTED:
                task_result = "aborted"
            else if goal_state == GoalStatus.PREEMPTED:
                task_result = "preempted"

            t_after = rospy.Time.now()
            rate.sleep()

        # Result of executing the action 
        return task_result

class SmachServer():
    def __init__(self):
        self.task_sm = None
        self.tasks = Queue()
        self.tasks_run = 0;

        # Add states to the queue to build the sm
        rospy.Service("/task_executor/" + "add_state", AddTask, self.add_state_srv)
       
        # Run loop
        self.run_sm()

    def reset_sm(self):
        self.task_sm = None

    def add_state_srv(self, TaskReq):
        self.tasks.put(TaskReq.task)

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
            init_transition = 'TASK_EXECUTION'
            smach.StateMachine.add('TASK_INITIALISATION', TaskInitialisation(self), transitions={'succeeded': 'TASK_EXECUTION'})
            
            # Final task outcomes
            smach.StateMachine.add('TASK_SUCCEEDED', TaskSucceeded(self), transitions={'succeeded':'succeeded'})
            smach.StateMachine.add('TASK_CANCELLED', TaskCancelled(self), transitions={'preempted':'preempted'})
            smach.StateMachine.add('TASK_FAILED', TaskFailed(self), transitions={'aborted':'aborted'})
            smach.StateMachine.add('TASK_EXECUTION', TaskExecution(self), 
                transitions={'preempted':'TASK_CANCELLED', 'aborted':'TASK_FAILED', 'succeeded':'TASK_SUCCEEDED'})

        # Execute SM
        self.task_sm.execute()

    def run_sm(self):
        r = rospy.Rate(1)

        # Construct the state machine from the list of tasks
        while not rospy.is_shutdown():
            self.tasks_run += 1;
            try:
                task_next = self.tasks.get(False)
                rospy.loginfo("Executing task %s", self.tasks_run)
                self.execute_task(task_next)
            except Empty, e:
                # If final state reached, add IDDLE state
                rospy.loginfo("Waiting for new tasks")
                pass

            r.sleep()             


if __name__ == '__main__':
    rospy.init_node('smach_state_machine')
    sm_server = SmachServer()
