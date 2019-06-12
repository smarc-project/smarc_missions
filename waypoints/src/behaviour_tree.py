# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviour tree that iterates 
# over a sequence of waypoints.

import py_trees_ros as ptr, py_trees as pt, rospy
from reactive_sequence import RSequence
from behaviours import *


class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self, plan=None):

        # set the plan
        self.plan = plan

        # safety
        b0 = pt.composites.Selector(children=[
            Counter(20, "Safe?"),
            Counter(20, "Become safe!")
        ])

        # do mission
        b1 = RSequence(children=[
            pt.composites.Selector(children=[
                Counter(20, "At waypoint?"),
                Counter(20, "Go to waypoint!")
            ]),
            Counter(20, "Update current waypoint!")
        ])


        # become behaviour tree
        super(BehaviourTree, self).__init__(
            RSequence(children=[b0, b1])
        )

        # execute the tree
        self.setup(timeout=100)
        while not rospy.is_shutdown():
            self.tick_tock(100)


if __name__ == "__main__":

    import numpy as np

    # n random waypoint plan
    n = 5
    plan = [
        {
            "id": _,
            "x": np.random.uniform(-100, 100),
            "y": np.random.uniform(-100, 100),
            "z": np.random.uniform(0, 100),
            "wait": np.random.uniform(0, 10)
        }
        for _ in range(n)
    ]

    print(plan)
    # execute a behaviour tree with the plan
    rospy.init_node('behaviour_tree')
    try:
        BehaviourTree(plan)
    except rospy.ROSInterruptException:
        pass