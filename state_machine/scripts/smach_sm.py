#!/usr/bin/env python

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
from smarc_msgs.msg import SMTask
from smarc_msgs.srv import AddTask
import threading

import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy


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

        # Create action client and goal dynamically. Based on mongodb
        (action_string, goal_string) = self.get_task_types(userdata.task.action_topic)
        action_clz = dc_util.load_class(dc_util.type_to_class_string(action_string))
        rospy.loginfo("Action string %s and goal string %s", action_string, goal_string)

        goal_clz = dc_util.load_class(dc_util.type_to_class_string(goal_string))
        argument_list = self.get_arguments(userdata.task.action_arguments)
        mb_goal = goal_clz(*argument_list)         

        # Create action client and wait for server
        self.act_client = actionlib.SimpleActionClient(userdata.task.action_topic, action_clz)
        rospy.loginfo("Waiting for server %s with action class %s", userdata.task.action_topic, action_clz)
        self.act_client.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Action server connected!")

        # Sends the goal to the action server.
        self.act_client.send_goal(mb_goal)
        rospy.loginfo("Goal sent!")

        # Check periodically state of AUV to preempt goal when needed
        t_before = rospy.Time.now()
        rate = rospy.Rate(1)
        task_result = "running"
        do_cancel = False
        while not rospy.is_shutdown() and task_result == "running":
            # Check pose of AUV  and compare to goal for terminating condition
            t_after = rospy.Time.now()
            if t_after - t_before > userdata.task.max_duration:
                rospy.loginfo("Time limit reached")
                do_cancel = True
                task_result = "preempted"
                break

            if self.end_condition(mb_goal):
                rospy.loginfo("Success!")
                do_cancel = True
                task_result = "succeeded"
                break

            # Check state from server side
            goal_state = self.act_client.get_state()
            if goal_state == GoalStatus.REJECTED or goal_state == GoalStatus.ABORTED:
                task_result = "aborted"
                rospy.loginfo("Action aborted!")
            elif goal_state == GoalStatus.PREEMPTED:
                rospy.loginfo("Action preempted!")                
                task_result = "preempted"
            elif goal_state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Action returned success!")
                task_result = "succeeded"


            rate.sleep()

        if do_cancel:
            self.act_client.cancel_all_goals()
            while not rospy.is_shutdown() and goal_state != GoalStatus.PREEMPTED:
                goal_state = self.act_client.get_state()
                rospy.loginfo("Waiting for server to preempt task")
                rospy.Rate(0.5).sleep()


        rospy.loginfo("task_result %s", task_result)

        # Result of executing the action 
        return task_result
        # return "succeeded"

    def get_arguments(self, argument_list):
        return map(self.instantiate_from_string_pair, argument_list)


    def instantiate_from_string_pair(self, string_pair):
        # rospy.loginfo("SMTask string %s", SMTask.STRING_TYPE)
        # rospy.loginfo("Type recevied %s", string_pair.first)
        if string_pair.string_array[0] == SMTask.STRING_TYPE:
            return string_pair.string_array[1]
        elif string_pair.string_array[0] == SMTask.INT_TYPE:
            return int(string_pair.string_array[1])
        elif string_pair.string_array[0] == SMTask.FLOAT_TYPE:
            return float(string_pair.string_array[1])     
        elif string_pair.string_array[0] == SMTask.TIME_TYPE:
            return rospy.Time.from_sec(float(string_pair.string_array[1]))
        elif string_pair.string_array[0] == SMTask.DURATION_TYPE:
            return rospy.Duration.from_sec(float(string_pair.string_array[1]))
        elif string_pair.string_array[0] == SMTask.BOOL_TYPE:   
            return string_pair.string_array[1] == 'True'
        elif string_pair.string_array[0] == SMTask.POSE_STAMPED_TYPE:   
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = string_pair.string_array[1]
            pose_stamped.pose.position.x = float(string_pair.string_array[2]) 
            pose_stamped.pose.position.y = float(string_pair.string_array[3]) 
            pose_stamped.pose.position.z = float(string_pair.string_array[4]) 
            pose_stamped.pose.orientation.x = float(string_pair.string_array[5]) 
            pose_stamped.pose.orientation.y = float(string_pair.string_array[6]) 
            pose_stamped.pose.orientation.z = float(string_pair.string_array[7]) 
            pose_stamped.pose.orientation.w = float(string_pair.string_array[8]) 
            return pose_stamped            
        else:
            # msg = self.msg_store.query_id(string_pair.second, string_pair.first)[0]
            # # print msg
            # if msg == None:
            raise RuntimeError("No matching object for id %s of type %s" % (string_pair.string_array[1], string_pair.string_array[0]))
            # return msg


    def get_task_types(self, action_name):
        """ 
        Returns the type string related to the action string provided.
        """
        rospy.logdebug("task action provided: %s", action_name)
        topics = rospy.get_published_topics(action_name)
        for [topic, type] in topics:            
            if topic.endswith('feedback'):
                return (type[:-8], type[:-14] + 'Goal')
        raise RuntimeError('No action associated with topic: %s'% action_name)


    def end_condition(self, mb_goal):

        try:
            (trans, rot) = NodeState.listener.lookupTransform("/world", NodeState.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

        # Check distance to navigation goal
        start_pos = np.array(trans)
        end_pos = np.array([mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y, mb_goal.target_pose.pose.position.z])
        if np.linalg.norm(start_pos - end_pos) < NodeState.goal_tolerance:
            rospy.loginfo("Reached goal!")
            return True
            
        return False

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
        return self.tasks_run

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
