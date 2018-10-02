import rospy
from smarc_planning_msgs.msg import ConditionalAction

def add_duration_argument(task, duration_arg):
    task.max_duration = rospy.Duration(duration_arg)
