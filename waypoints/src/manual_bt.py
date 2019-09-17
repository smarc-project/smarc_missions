#!/usr/bin/python
# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviour tree that iterates through
# an apriori sequence of waypoints

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours import Sequence, Safe
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal

class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self, plan):

        # the blackboard
        self.bb = pt.blackboard.Blackboard()

        # set initial variables
        self.bb.set("plan", plan)
        self.bb.set("n_waypoints", len(plan))
        self.bb.set("waypoint_i", 0)

        # safety branch
        s = Safe()

        # mission execution - GoToWayPoint is Harsha's action server
        me = pt.composites.Selector(children=[
            AtFinalWaypoint(),
            Sequence(children=[
                GoToWayPoint(),
                SetNextWaypoint()
                ])
            ])

        # default behaviour
        db = pt.behaviours.Running(name="Shutting down!")

        # assemble behaviour tree
        ptr.trees.BehaviourTree.__init__(self, Sequence(children=[
            s, me, db
        ]))

class AtFinalWaypoint(pt.behaviour.Behaviour):

    def __init__(self):

        # blackboard access
        self.bb = pt.blackboard.Blackboard()

        # become behaviour
        pt.behaviour.Behaviour.__init__(self, "At final waypoint?")

    def update(self):

        # current status
        i = self.bb.get("waypoint_i")
        n = self.bb.get("n_waypoints")
        self.feedback_message = "Waypoint {} of {}".format(i, n)

        # react to result
        return pt.common.Status.SUCCESS if i == n else pt.common.Status.FAILURE

class SetNextWaypoint(pt.behaviour.Behaviour):

    def __init__(self):

        # blackboard access
        self.bb = pt.blackboard.Blackboard()

        # become behaviour
        pt.behaviour.Behaviour.__init__(self, "Set next waypoint")

    def update(self):

        # set current waypoint to the next one
        self.bb.set("waypoint_i", self.bb.get("waypoint_i") + 1)
        return pt.common.Status.RUNNING

class GoToWayPoint(ptr.actions.ActionClient):

    def __init__(self):

        # blackboard access
        self.bb = pt.blackboard.Blackboard()

        # become action client
        ptr.actions.ActionClient.__init__(
            self,
            name="Go to waypoint",
            action_spec=MoveBaseAction,
            action_goal=None,
            action_namespace="/p2p_planner",
        )

    def initialise(self):

        # get waypoint
        i = self.bb.get("waypoint_i")
        wp = self.bb.get("plan")[i]

        # construct the message
        self.action_goal = MoveBaseActionGoal()
        self.action_goal.goal.target_pose.pose.position.x = wp[0]
        self.action_goal.goal.target_pose.pose.position.y = wp[1]
        self.action_goal.goal.target_pose.pose.position.z = wp[2]

        self.sent_goal = False

if __name__ == "__main__":

    # initialise node
    rospy.init_node("manual_behaviour_tree")

    # waypoints
    wps = [(0,0,0), (1,1,0), (2,2,0)]

    # execute behaviour tree
    try:
        bt = BehaviourTree(wps)
        bt.setup(timeout=10)
        while not rospy.is_shutdown():
            bt.tick_tock(1)
            print(pt.display.print_ascii_tree(bt))
    except rospy.ROSInterruptException:
        pass