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

        # safety
        s0 = Counter(20, name='Safe?')
        s1 = pt.behaviours.Running(name="Safety action!")
        s = pt.composites.Selector(children=[s0, s1])

        # system preperation
        sp0 = Counter(20, name='Continue command recieved?')
        sp1 = pt.behaviours.Running(name='Preparing system!')
        sp = pt.composites.Selector(children=[sp0, sp1])

        # mission synchronisation
        ms0 = Counter(20, name='Mission synchronised?')
        ms1 = pt.behaviours.Running(name='Synchronising mission!')
        ms = pt.composites.Selector(children=[ms0, ms1])

        # mission execution
        me0 = pt.blackboard.CheckBlackboardVariable(
            "At final waypoint?",
            variable_name="goal_waypoint",
            expected_value=self.bb.get("n_waypoints")
        )
        me1 = Counter(5, name="At waypoint", reset=True)
        me2 = pt.behaviours.Running(name="Going to waypoint")
        me3 = SetNextWaypoint()
        me = pt.composites.Selector(children=[me1, me2])
        me = Sequence(children=[me, me3])
        me = pt.composites.Selector(children=[me0, me])

        # mission finalisation
        self.bb.set("mission_done", False)
        mf0 = pt.blackboard.CheckBlackboardVariable(
            "Mission done?",
            variable_name="mission_done",
            expected_value=True
        )
        mf1 = Counter(20, name="At surface?")
        mf2 = pt.behaviours.Running(name="Going to surface!")
        mf3 = Counter(20, name="Payload off?")
        mf4 = pt.behaviours.Running(name="Shutting down!")
        mf = Sequence(children=[
            pt.composites.Selector(children=[mf1, mf2]),
            pt.composites.Selector(children=[mf3, mf4]),
            pt.blackboard.SetBlackboardVariable(
                name="Mission done!",
                variable_name="mission_done",
                variable_value=True
            )
        ])
        mf = pt.composites.Selector(children=[mf0, mf])

        # become behaviour tree
        super(BehaviourTree, self).__init__(Sequence(children=[
            s, sp, ms, me, mf
        ]))

        # execute the tree
        self.setup(timeout=1000)
        while not rospy.is_shutdown():
            self.tick_tock(100)


if __name__ == "__main__":

    # execute a behaviour tree with the plan
    rospy.init_node('behaviour_tree')
    try:
        BehaviourTree('betterplan.json')
    except rospy.ROSInterruptException:
        pass