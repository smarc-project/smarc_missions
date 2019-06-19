# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours to use within a behaviour tree.
# https://arxiv.org/abs/1811.00426

import py_trees as pt

class Counter(pt.behaviour.Behaviour):

    # A simple counter

    def __init__(self, n, name='Counter', reset=False):

        # count
        self.i = 0
        self.n = n

        # resetting
        self.reset = reset

        # become a behaviour
        super(Counter, self).__init__(name)

    def update(self):

        # increment the count
        self.i += 1

        # react to the result
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS

    def terminate(self, status):
        self.i = 0 if status == pt.common.Status.SUCCESS and self.reset else self.i

class SetNextWaypoint(pt.behaviour.Behaviour):

    def __init__(self):

        # blackboard
        self.bb = pt.blackboard.Blackboard()

        # become a behaviour
        super(SetNextWaypoint, self).__init__("Set next waypoint!")

    def update(self):

        # set current waypoint to the next one
        self.bb.set("goal_waypoint", self.bb.get("goal_waypoint") + 1)
        return pt.common.Status.RUNNING

        