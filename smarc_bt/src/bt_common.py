#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

# common definitions and global variables

import py_trees as pt
import py_trees_ros as ptr
import rospy

import copy # used in ReadTopic

from smarc_bt.msg import CBFList, CBFItem
import common_globals

###############################################################
# GENERIC TREE NODES AND SUCH
###############################################################

class CBFCondition(object):
    """
    An object for creating conditions that are also control barrier
    functions. This allows us to send a set of CBFs to actions to use.
    Instanciate this object in your normal ros-y condition nodes.
    Use the A_ClearCBFs action above at the beginning of your tree if you use any of these.
    Otherwise your actions might run the SAME cbfs with conflicting limits and just explode.

    And then in the update() method of C_XY, if the update() is about to return success,
    call the cbf_update method too.

    The system works like this:
        At the beginning of the tree, a simple action calls cbf_update(reset=True).
            This publishes an empty cbf_list message.
        Afterwards, every condition that inherits this object, when they succeed,
        calls cbf_update().
            This reads reads the current cbf_bt/active_limits topics, appends
            its own cbf_item to the list and publishes the new, longer cbf_list.
        At any point, an action that reads the cbf_list object gets an ordered list
        of all succeeded conditions that ran before it.
    """
    def __init__(self, update_func, limit_type, limit_value, checked_field_topic='', checked_field_name=''):

        self.update_func = update_func

        # the ros message can not have Nones in it.
        if checked_field_name is None:
            checked_field_name = ''
        if checked_field_topic is None:
            checked_field_topic = ''
        # this item does not change, so we can cache it
        self.cbf_item = CBFItem()
        self.cbf_item.checked_field_topic = checked_field_topic
        self.cbf_item.checked_field_name = checked_field_name
        self.cbf_item.limit_type = limit_type
        self.cbf_item.limit_value = limit_value

        self._latest_list = None
        self._this_is_first_cond_in_tree = False

        self.cbf_pub = rospy.Publisher(common_globals.CBF_BT_TOPIC, CBFList, queue_size=1)
        self.cbf_sub = rospy.Subscriber(common_globals.CBF_BT_TOPIC, CBFList, callback=self.cbf_list_cb)

    def cbf_list_cb(self, cbf_list):
        self._latest_list = cbf_list


    def update(self):
        return_status = self.update_func()

        if self._latest_list is None:
            # the callback was never called, if it was, this would not be None
            # it would have been at worst an empty message object.
            # this means this condition is the first ever condition
            # in the tree to be caled.
            self._this_is_first_cond_in_tree = True

        # create a new empty list
        # for sure if we are the first condition
        if self._this_is_first_cond_in_tree:
            cbf_list = CBFList()
        else:
            cbf_list = self._latest_list

        # if the child condition udpate method is successful, add this cbf to the list
        if return_status == pt.Status.SUCCESS:
            cbf_list.cbf_items.append(self.cbf_item)
        # otherwise leave it as is
        # this happens when the first condition of the list fails
        # either way, publish this filled or empty list
        self.cbf_pub.publish(cbf_list)

        # and return the original update's return
        return return_status



class ReadTopic(pt.behaviour.Behaviour):
    """
    A simple subscriber that returns SUCCESS all the time,
    even if there is no data in the topic.
    Puts the data into BB if there is any data
    Same usage as ptr.subscribers.ToBlackboard, except it doesnt return RUNNING when
    there is no data.

    mostly copied from the "ToBlackboard" behaviour of ptr
    """
    def __init__(self, name, topic_name, topic_type, blackboard_variables):
        self.bb = pt.blackboard.Blackboard()
        self.blackboard_variables = blackboard_variables
        self.last_read_value = None

        self.topic_name = topic_name
        self.topic_type = topic_type
        self.subs = None
        self.msg = None

        super(ReadTopic, self).__init__(name)

    def setup(self, timeout):
        self.subs = rospy.Subscriber(self.topic_name, self.topic_type, self._cb, queue_size=2)
        return True

    def _cb(self, msg):
        self.msg = msg

    def update(self):
        if self.msg is not None:
            self.last_read_value = copy.copy(self.msg)
            for k,v in self.blackboard_variables.iteritems():
                if v is None:
                    self.bb.set(k, self.msg, overwrite=True)
                else:
                    fields = v.split(".")
                    value = copy.copy(self.msg)
                    for field in fields:
                        value = getattr(value, field)
                        self.bb.set(k, value, overwrite=True)

        self.feedback_message = "Last read:"+str(self.last_read_value)
        return pt.Status.SUCCESS





class CheckBlackboardVariableValue(pt.behaviour.Behaviour):
    """
    re-implementation of a future node that we do not have in our version of
    the py-trees library.
    Checks that the given blackboard variable has the given value.
    returns succcess if there IS a value, and it is what we expected,
    otherwiser FAILURE
    """
    def __init__(self, variable_name, expected_value, name):
        self.bb = pt.blackboard.Blackboard()
        self.variable_name = variable_name
        self.expected_value = expected_value

        super(CheckBlackboardVariableValue, self).__init__(name)

    def update(self):
        current_value = self.bb.get(self.variable_name)
        if current_value is None or current_value != self.expected_value:
            self.feedback_message = "Received value:"+str(current_value)
            return pt.Status.FAILURE

        return pt.Status.SUCCESS




class Sequence(pt.composites.Selector):

    """
    Reactive sequence overidding sequence with memory, py_trees' only available sequence.
    """

    def __init__(self, name="Sequence", children=None, blackbox_level=None):
        super(Sequence, self).__init__(name=name, children=children, blackbox_level=blackbox_level)

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
