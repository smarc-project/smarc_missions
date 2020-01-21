#!/usr/bin/python
# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviour tree that iterates
# over a sequence of waypoints
# sent from Neptus...
# Refer to behaviours.py for
# behaviour specifications.

import py_trees as pt, py_trees_ros as ptr, rospy, json
from behaviours import Sequence, Safe, SynchroniseMission, AtFinalWaypoint, GoToWayPoint, SetNextWaypoint

class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self, plan_db_ns):
        """
        SEQ[
            Safe
            Synch Mission
            FB[
                AtFinalWP
                SEQ[
                    GoToWP
                    SetNextWP
                   ]
              ]
            Idle
            ]
        """

        # blackboard access
        self.bb = pt.blackboard.Blackboard()

        # safety branch
        s = Safe(plan_db_ns)

        # mission synchronisation
        ms = SynchroniseMission(plan_tpc=plan_db_ns + '/plan_db')

        # mission execution
        me = pt.composites.Selector(children=[
            AtFinalWaypoint(),
            Sequence(children=[
                GoToWayPoint(plan_db_ns),
                SetNextWaypoint()
            ])
        ])

        # default behaviour
        db = pt.behaviours.Running(name="Doing nothing!")

        # assemble behaviour tree
        ptr.trees.BehaviourTree.__init__(self, Sequence(children=[
            s, ms, me, db
        ]))

if __name__ == "__main__":

    # initialise node
    rospy.init_node("neptus_bt")
    
    # get the namespace for the topics
    plan_db_ns = rospy.get_param("~system_name")

    # execute behaviour tree
    try:
        bt = BehaviourTree(plan_db_ns)
        bt.setup(timeout=10)
        while not rospy.is_shutdown():
            #bt.tick_tock(1, post_tick_handler=lambda t: pt.display.print_ascii_tree(bt.root, show_status=True))
            bt.tick_tock(1)
    except rospy.ROSInterruptException:
        pass
