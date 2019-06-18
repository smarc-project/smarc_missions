# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviour tree that iterates 
# over a sequence of waypoints.

import py_trees_ros as ptr, py_trees as pt, rospy, json
from custom_behaviours import *
from behaviours import *


class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self, plan=None):

        # the blackboard
        self.bb = pt.blackboard.Blackboard()

        # set the plan
        if isinstance(plan, dict):
            self.bb.set('plan', plan)
        else:
            try:
                with open(plan, 'r') as f:
                    plan = json.load(f)
                    self.bb.set('plan', plan)
            except:
                print("Can't find your plan!")

        # set current waypoint
        self.bb.set("goal_waypoint", 0)

        # set number of waypoints
        self.bb.set("n_waypoints", len(plan))


        '''
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
        '''

        '''
        # become behaviour tree
        super(BehaviourTree, self).__init__(
            Sequence(children=[b0, b1])
        )

        # execute the tree
        self.setup(timeout=100)
        while not rospy.is_shutdown():
            self.tick_tock(100)
        '''


if __name__ == "__main__":

    tree = BehaviourTree('betterplan.json')
    #print(tree.plan)

    '''
    # execute a behaviour tree with the plan
    rospy.init_node('behaviour_tree')
    try:
        BehaviourTree(plan)
    except rospy.ROSInterruptException:
        pass
    '''