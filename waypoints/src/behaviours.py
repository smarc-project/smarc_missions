# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours to use within a behaviour tree.
# https://arxiv.org/abs/1811.00426

import py_trees as pt, py_trees_ros as ptr, itertools, std_msgs.msg, copy, json, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geodesy.utm import fromLatLong


import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

class Sequence(pt.composites.Selector):

    """
    Reactive sequence overidding sequence with memory, py_trees' only available sequence.
    """

    def __init__(self, name="Sequence", children=None):
        super(Sequence, self).__init__(name=name, children=children)

    def tick(self):
        """
        Run the tick behaviour for this selector. Note that the status
        of the tick is always determined by its children, not
        by the user customised update function.
        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # Required behaviour for *all* behaviours and composites is
        # for tick() to check if it isn't running and initialise
        if self.status != pt.common.Status.RUNNING:
            # selectors dont do anything specific on initialisation
            #   - the current child is managed by the update, never needs to be 'initialised'
            # run subclass (user) handles
            self.initialise()
        # run any work designated by a customised instance of this class
        self.update()
        previous = self.current_child
        for child in self.children:
            for node in child.tick():
                yield node
                if node is child:
                    if node.status == pt.common.Status.RUNNING or node.status == pt.common.Status.FAILURE:
                        self.current_child = child
                        self.status = node.status
                        if previous is None or previous != self.current_child:
                            # we interrupted, invalidate everything at a lower priority
                            passed = False
                            for child in self.children:
                                if passed:
                                    if child.status != pt.common.Status.INVALID:
                                        child.stop(pt.common.Status.INVALID)
                                passed = True if child == self.current_child else passed
                        yield self
                        return
        # all children succeded, set succed ourselves and current child to the last bugger who failed us
        self.status = pt.common.Status.SUCCESS
        try:
            self.current_child = self.children[-1]
        except IndexError:
            self.current_child = None
        yield self

class SetBlackboardVariable(pt.behaviours.Running):
    """
    Set the specified variable on the blackboard.
    Usually we set variables from inside other behaviours, but can
    be convenient to set them from a behaviour of their own sometimes so you
    don't get blackboard logic mixed up with more atomic behaviours.
    Args:
        name (:obj:`str`): name of the behaviour
        variable_name (:obj:`str`): name of the variable to set
        variable_value (:obj:`any`): value of the variable to set
    .. todo:: overwrite option, leading to possible failure/success logic.
    """
    def __init__(self,
                 name="Set Blackboard Variable",
                 variable_name="dummy",
                 variable_value=None
                 ):
        """
        :param name: name of the behaviour
        :param variable_name: name of the variable to set
        :param value_name: value of the variable to set
        """
        super(SetBlackboardVariable, self).__init__(name)
        self.variable_name = variable_name
        self.variable_value = variable_value

    def initialise(self):
        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard.set(self.variable_name, self.variable_value, overwrite=True)

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

        # blackboard access
        self.bb = pt.blackboard.Blackboard()

        # become behaviour
        pt.behaviour.Behaviour.__init__(self, "Set next waypoint")

    def update(self):

        # set current waypoint to the next one
        self.bb.set("waypoint_i", self.bb.get("waypoint_i") + 1)
        return pt.common.Status.RUNNING

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
            
class GoTo(pt.behaviour.Behaviour):

    """
    Publishes to relevant topics while preconditions are met.
    NOTE: basic publishing until go to action server is available
    """

    def __init__(self):

        # blackboard
        self.bb = pt.blackboard.Blackboard()

        # publishers
        self.pitch = rospy.Publisher(
            '/pitch_setpoint',
            std_msgs.msg.Float64,
            queue_size=100
        )
        self.depth = rospy.Publisher(
            '/depth_setpoint',
            std_msgs.msg.Float64,
            queue_size=100
        )

        # become a behaviour
        super(GoTo, self).__init__("Go to waypoint!")

    def update(self):
        
        # current waypoint
        i = self.bb.get("goal_waypoint")

        # pitch and depth goals
        pitch = float(self.bb.get('plan')[i]['data']['pitch'])
        depth = float(self.bb.get('plan')[i]['data']['z'])

        # publish
        self.pitch.publish(std_msgs.msg.Float64(pitch))
        self.depth.publish(std_msgs.msg.Float64(depth))

        # feedback
        self.feedback_message = "Waypoint {} | pitch={}, depth={}".format(i, pitch, depth)

        # always running if preconditions are met
        return pt.common.Status.RUNNING

class SynchroniseMission(ptr.subscribers.Handler):

    '''
    - Returns running until /plan_db recieves a 
    message.
    - Set's the plan and the number of waypoints, 
    returning success after /plan_db recieves a
    message.
    - Returns success indefinitely thereafter.
    '''

    def __init__(self, plan_tpc='/plan_db'):

        # blackboard
        self.bb = pt.blackboard.Blackboard()

        # first execution
        self.first = True

        # become a behaviour
        super(SynchroniseMission, self).__init__(
            name="Synchronise mission!",
            topic_name=plan_tpc,
            topic_type=std_msgs.msg.String,   
        )
        
        # create a markerArray publisher
        # TODO change topic
        self.marker_array_pub = rospy.Publisher('/rviz_marker_array', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()

    def update(self):

        with self.data_guard:

            # first time - checking length because neptus sends empty message sometimes, not sure which times.... :'(
            if (self.msg ==  None or len(str(self.msg)) < 135) and self.first:
                self.feedback_message = "Waiting for a plan"
                return pt.common.Status.RUNNING

            # recieved plan
            if isinstance(self.msg, std_msgs.msg.String):
                self.feedback_message = "Recieved new plan"
                # format the message to .json
                plan = self.clean(str(self.msg))
                self.bb.set("plan", plan)
                self.bb.set("n_waypoints", len(plan))
                self.bb.set("waypoint_i", 0)
                self.first = False
                
                #also publish the points into for rviz
                for i, ptn in enumerate(plan):
                    marker = Marker()
                    marker.ns = '/marker_array'
                    marker.id = i
                    marker.action = 0
                    marker.type = 3 #cylinder
                    pose = Pose()
                    pose.position.x = ptn[0]
                    pose.position.y = ptn[1]
                    pose.position.z = ptn[2]
                    marker.pose = pose
                    marker.color.a = 255
                    marker.color.r = 255
                    marker.color.g = 0
                    marker.color.b = 255
                    marker.header.frame_id = '/world'
                    self.marker_array.markers.append(marker)
                    
                self.marker_array_pub.publish(self.marker_array)
                    
                return pt.common.Status.SUCCESS

            # otherwise
            else:
                self.feedback_message = "No new plan"
                return pt.common.Status.SUCCESS      

    @staticmethod
    def clean(plan):

        # load the pretty json file
        #f = open(fname, 'r').read()
        f = plan

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
        f = f.split(',"transitions"')[0]

        # convert to json
        f = json.loads(f)

        # convert lat lon to utm
        f = [fromLatLong(float(d['data']['lat']), float(d['data']['lon'])) for d in f]

        # convert utm to point
        f = [d.toPoint() for d in f]

        # convert point to xyz NOTE: convert to utm local?
        f = [(d.x, d.y, d.z) for d in f]

        # save
        '''
        if sfname is not None:
            with open(sfname, 'w') as sf:
                json.dump(f, sf, sort_keys=True, indent=4)
        '''

        # return the json dictionary
        return f


class Safe(ptr.subscribers.Handler):

    '''
    Returns success as long as there is not
    any message recieved at /abort.
    '''

    def __init__(self):

        # become a behaviour
        super(Safe, self).__init__(
            name="Safe?",
            topic_name="/abort",
            topic_type=std_msgs.msg.Empty
        )

    def update(self):

        # do not abort
        if self.msg == None:
            self.feedback_message = "Everything is okay"
            return pt.common.Status.SUCCESS
        
        # abort
        else:
            self.feedback_message = "Mission aborted!"
            return pt.common.Status.FAILURE

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
        self.action_goal = MoveBaseGoal()
        self.action_goal.target_pose.pose.position.x = wp[0]
        self.action_goal.target_pose.pose.position.y = wp[1]
        self.action_goal.target_pose.pose.position.z = wp[2]

        self.sent_goal = False

"""
class DataPublisher(pt.behaviour.Behaviour):

    def __init__(self):

        # blackboard
        self.bb = pt.blackboard.Blackboard()

        # initialise the blackboard
        self.bb.set("ready", 0)
        self.bb.set("initialising", 0)
        self.bb.set("executing", 0)

        # topic
        self.pub = rospy.Publisher(
            '/plan_cotrol_state',
            imc_ros_bridge.msg.PlanControl,
            queue_size=100
        )

        # become behaviour
        super(DataPublisher, self).__init__("Data publisher")

    def update(self):

        # instantiate message
        msg = imc_ros_bridge.msg.PlanControl()

        # add relevant things from blackboard
        msg.READY = std_msgs.msg.UInt8(self.bb.get("ready"))
        msg.INITIALIZING = std_msgs.msg.UInt8(self.bb.get("initialising"))
        msg.EXECUTING = std_msgs.msg.UInt8(self.bb.get("executing"))

        # NOTE: add more to message

        # publish it!
        self.pub.publish(msg)

        return pt.common.Status.RUNNING

"""
