# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours to use within a behaviour tree.
# https://arxiv.org/abs/1811.00426

import py_trees as pt

class Counter(pt.behaviour.Behaviour):

    # A simple counter

    def __init__(self, n, name='Counter'):

        # count
        self.i = 0
        self.n = n

        # become a behaviour
        super(Counter, self).__init__(name)

    def update(self):

        # increment the count
        self.i += 1

        # react to the result
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS

class GoTo(pt.behaviour.Behaviour):

    def __init__(self, name='Go to!'):

        # blackboard access
        self.bb = pt.blackboard.Blackboard()

        # become a behaviour
        super(GoTo, self).__init__(name)

    def update(self):

        # get current goal index
        self.i = self.bb.get('goal_waypoint')

        