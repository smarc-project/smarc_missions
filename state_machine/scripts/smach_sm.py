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
from smarc_msgs.msg import SMTask, EmptyActionGoal
from smarc_msgs.srv import AddTask
import threading

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
        dictionary = ast.literal_eval(userdata.task_struct[0].action_arguments)
        mb_goal = message_converter.convert_dictionary_to_ros_message(action_tuple[1], dictionary)
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
            elif goal_state == GoalStatus.PREEMPTED:
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

    # def get_arguments(self, argument_list):
    #     return map(self.instantiate_from_string_pair, argument_list)


    # def instantiate_from_string_pair(self, string_pair):
    #     # rospy.loginfo("SMTask string %s", SMTask.STRING_TYPE)
    #     # rospy.loginfo("Type recevied %s", string_pair.first)
    #     if string_pair.string_array[0] == SMTask.STRING_TYPE:
    #         return string_pair.string_array[1]
    #     elif string_pair.string_array[0] == SMTask.INT_TYPE:
    #         return int(string_pair.string_array[1])
    #     elif string_pair.string_array[0] == SMTask.FLOAT_TYPE:
    #         return float(string_pair.string_array[1])     
    #     elif string_pair.string_array[0] == SMTask.TIME_TYPE:
    #         return rospy.Time.from_sec(float(string_pair.string_array[1]))
    #     elif string_pair.string_array[0] == SMTask.DURATION_TYPE:
    #         return rospy.Duration.from_sec(float(string_pair.string_array[1]))
    #     elif string_pair.string_array[0] == SMTask.BOOL_TYPE:   
    #         return string_pair.string_array[1] == 'True'
    #     elif string_pair.string_array[0] == SMTask.POSE_STAMPED_TYPE:   
    #         pose_stamped = PoseStamped()
    #         pose_stamped.header.frame_id = string_pair.string_array[1]
    #         pose_stamped.pose.position.x = float(string_pair.string_array[2]) 
    #         pose_stamped.pose.position.y = float(string_pair.string_array[3]) 
    #         pose_stamped.pose.position.z = float(string_pair.string_array[4]) 
    #         pose_stamped.pose.orientation.x = float(string_pair.string_array[5]) 
    #         pose_stamped.pose.orientation.y = float(string_pair.string_array[6]) 
    #         pose_stamped.pose.orientation.z = float(string_pair.string_array[7]) 
    #         pose_stamped.pose.orientation.w = float(string_pair.string_array[8]) 
    #         return pose_stamped            
    #     else:
    #         raise RuntimeError("No matching object for id %s of type %s" % (string_pair.string_array[1], string_pair.string_array[0]))
            

    def get_task_types(self, action_name):
        result = ()
        topics = rospy.get_published_topics(action_name)
        for [topic, type] in topics:            
            if topic.endswith('feedback'):
                result = (type[:-8], type[:-14] + 'Goal')

        return result

        # raise RuntimeError('No action associated with topic: %s'% action_name)


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

        # ROS params
        NodeState.base_frame = rospy.get_param('~base_frame', "lolo_auv_1/base_link")
        NodeState.heading_offset = rospy.get_param('~heading_offsets', 5.)
        NodeState.goal_tolerance = rospy.get_param('~goal_tolerance', 5.)
        NodeState.add_task_srv = rospy.get_param('~add_task_srv', 5.)
        NodeState.add_tasks_srv = rospy.get_param('~add_tasks_srv', 5.)
        NodeState.listener = tf.TransformListener()

        # Add states to the queue to build the sm
        rospy.Service(NodeState.add_task_srv, AddTask, self.add_task_srv_cb)
        rospy.Service(NodeState.add_tasks_srv, AddTasks, self.add_tasks_srv_cb)

        # Run loop
        self.run_sm()

    def add_task_srv_cb(self, TaskReq):
        self.tasks.append(TaskReq.task)
        # return self.tasks_run
    
    def add_tasks_srv_cb(self, TasksReq):
        for task in TasksReq:
            self.add_task_srv_cb(task)
        
        # return self.tasks_run

    def execute_task(self, task):
        rospy.loginfo("Executing task %s", self.tasks_run)

        # Reset SM 
        self.task_sm = None
        act_client = None
        task_result_flag = ""
        # Create the state machine necessary to execute this task        
        self.task_sm = smach.StateMachine(['succeeded','aborted','preempted'])

        # Update the userdata with the new task and an action client to pass between states
        self.task_sm.userdata.task_struct = [task, act_client, task_result_flag]

        with self.task_sm:

            # Initialise task data
            smach.StateMachine.add('TASK_INITIALIZATION', TaskInitialization(), transitions={'succeeded': 'TASK_EXECUTION'})
            # Final task outcomes
            smach.StateMachine.add('TASK_SUCCEEDED', TaskSucceeded(), transitions={'succeeded':'succeeded'})
            smach.StateMachine.add('TASK_CANCELLED', TaskCancelled(), transitions={'preempted':'preempted'})
            smach.StateMachine.add('TASK_FAILED', TaskFailed(), transitions={'aborted':'aborted'})
            smach.StateMachine.add('TASK_PREEMPTING', TaskPreempting(), transitions={'preempted':'TASK_CANCELLED', 'succeeded':'TASK_SUCCEEDED'})
            smach.StateMachine.add('TASK_EXECUTION', TaskExecution(), 
                transitions={'preempted':'TASK_CANCELLED', 'preempting':'TASK_PREEMPTING', 'aborted':'TASK_FAILED', 'succeeded':'TASK_SUCCEEDED'})

        # Execute SM
        self.task_sm.set_initial_state(['TASK_INITIALIZATION'])
        self.task_sm.execute()

    def run_sm(self):
        r = rospy.Rate(1)

        # Construct the state machine from the list of tasks
        while not rospy.is_shutdown():
            if not len(self.tasks) == 0:
                task_next = self.tasks.pop(0)
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
