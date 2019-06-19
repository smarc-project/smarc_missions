# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviour tree that iterates 
# over a sequence of waypoints.

import py_trees_ros as ptr, py_trees as pt, rospy, json
from behaviours import *


class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self, plan=None):

        # the blackboard
        self.bb = pt.blackboard.Blackboard()

        # set the plan
        self.set_plan(plan)

        # safety NOTE: replace
        s0 = Counter(20, name='Safe?')
        s1 = pt.behaviours.Running(name="Safety action!")
        s = pt.composites.Selector(children=[s0, s1])

        # system preperation NOTE: replace
        sp0 = Counter(20, name='Continue command recieved?')
        sp1 = pt.behaviours.Running(name='Preparing system!')
        sp = pt.composites.Selector(children=[sp0, sp1])

        # mission synchronisation NOTE: replace
        #ms0 = Counter(20, name='Mission synchronised?')
        #ms1 = pt.behaviours.Running(name='Synchronising mission!')
        #ms = pt.composites.Selector(children=[ms0, ms1])
        ms = SynchroniseMission()

        # mission execution
        me0 = AtFinalWaypoint()
        me1 = Counter(5, name="At waypoint", reset=True)
        me2 = GoTo()
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

    @staticmethod
    def clean(fname, sfname=None):

        # load the pretty json file
        f = open(fname, 'r').read()

        # clean
        f = f.replace(' ', '')
        f = f.replace('\\n', '')
        f = f.replace('\\"', '"')
        f = f.replace('"\\', '"')
        f = f.replace('\\', '')

        # remove
        f = f.split(',"transitions":')[0]
        f = f.split('"maneuvers":')[1]
        f = f.replace('\n', '')

        # convert to json
        f = json.loads(f)

        # save
        if sfname is not None:
            with open(sfname, 'w') as sf:
                json.dump(f, sf, sort_keys=True, indent=4)

        # return the json dictionary
        return f

    def set_plan(self, plan):

        # if given json dict plan
        if isinstance(plan, dict):
            plan = plan

        # if given filename
        elif isinstance(plan, str):

            # if given nice json file
            try:
                with open(plan, 'r') as f:
                    plan = json.load(f)
            except:

                # if given bad json file
                try:
                    plan = self.clean(plan)
                except:
                    raise ValueError("Could not use or find your plan.")

        # if given something else
        else:
            raise ValueError("Must give either dictionary or filename.")

        # set the plan
        self.bb.set("plan", plan)
        
        # set current waypoint
        self.bb.set("goal_waypoint", 0)

        # set number of waypoints
        self.bb.set("n_waypoints", len(plan))


if __name__ == "__main__":

    # execute a behaviour tree with the plan
    rospy.init_node('behaviour_tree')
    try:
        BehaviourTree('betterplan.json')
    except rospy.ROSInterruptException:
        pass