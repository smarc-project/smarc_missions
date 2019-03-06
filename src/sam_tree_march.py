#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# 1 Mar 2019


from __future__ import print_function

import rospy, time
import actionlib

import functools, sys, time

from std_msgs.msg import Empty, Bool, Float64
from sam_march.msg import GenericStringAction

import py_trees as pt
import py_trees_ros as ptr

from reactive_seq import ReactiveSeq
import sam_behaviours
from sam_emergency import Emergency

if __name__ == '__main__':
    # a node that just keeps running once the tree is done
    idle = pt.behaviours.Running(name='Idle')

    #####################
    # DATA GATHERING SUBTREE
    #####################
    # behaviours that will read some topic from ros topics
    # will go under this
    topics2bb = pt.composites.Sequence("Topics to BB")

    # this will sub to '/abort' and write to 'abort' in the BB of the tree when an Empty is received
    emergency2bb = ptr.subscribers.EventToBlackboard(name='Emergency button',
                                                     topic_name='/abort',
                                                     variable_name='emergency')

    # returns running if there is no data to write until there is data
    # the dict here makes the behaviour write only the .data part of the whole message here
    # into the blackboard variable pitch
    # the clearing policy will not allow the bb variable to be cleared, it'll only be re-written
    # init_variables gives a dict for each bb variable and the value to init it with
    pitch2bb = ptr.subscribers.ToBlackboard(name='Pitch',
                                            topic_name='/feedback_pitch',
                                            topic_type=Float64,
                                            blackboard_variables={'pitch':'data'},
                                            initialise_variables={'pitch':0},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER)

    # same as pitch
    depth2bb = ptr.subscribers.ToBlackboard(name='Depth',
                                            topic_name='/feedback_depth',
                                            topic_type=Float64,
                                            blackboard_variables={'depth':'data'},
                                            initialise_variables={'depth':0},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER)


    # add these to the subtree responsible for data acquisition
    topics2bb.add_children([emergency2bb, pitch2bb, depth2bb])



    #####################
    # SAFETY SUBTREE
    #####################

    # if emergency is False, then we are safe.
    check_safe = pt.blackboard.CheckBlackboardVariable(name="Safe?",
                                                       variable_name='emergency',
                                                       expected_value=False)

    check_safety_tried = pt.blackboard.CheckBlackboardVariable(name="Safety action tried?",
                                                               variable_name='safety_tried',
                                                               expected_value=False)

    # use py_trees.behaviour.Behavior
    safety_action = ptr.actions.ActionClient(name='sam_emergency',
                                             action_spec=GenericStringAction,
                                             action_goal="",
                                             action_namespace='/sam_emergency')

    set_safety_tried = pt.blackboard.SetBlackboardVariable(name='Set safety tried',
                                                           variable_name='safety_tried',
                                                           variable_value=True)

    attempt_safety = pt.composites.Parallel(name='Attempt safety action')
    attempt_safety.add_children([safety_action, set_safety_tried])


    # tries the safety action once and then idles
    safety_fb = pt.composites.Selector(name='Safety')
    safety_fb.add_children([check_safe, check_safety_tried, attempt_safety, idle])


    #####################
    # MISSION SUBTREE
    #####################

    mission_fb = pt.composites.Selector(name='Mission')

    # return SUCCESS if the mission_complete flag is True
    mission_not_complete = pt.blackboard.CheckBlackboardVariable(name='Mission complete?',
                                                                 variable_name='mission_complete',
                                                                 expected_value=True)

    # first check is to see if the mission is complete
    mission_fb.add_child(mission_not_complete)

    # do the mission, if it succeeds, set flag
    mission_exec = ReactiveSeq(name='Mission execution')

    ##############################################################################################
    # ACTUAL MISSION DONE HERE
    ##############################################################################################
    # this mission should be a subtree of behaviour or action client
    execute_mission = sam_behaviours.some_mission(name="Execute mission")
    ##############################################################################################

    set_mission_complete = pt.blackboard.SetBlackboardVariable(name='Set mission complete',
                                                               variable_name='mission_complete',
                                                               variable_value=True)
    # add in order
    #  mission_exec.add_children([execute_mission, set_mission_complete])
    mission_exec.add_child(execute_mission)
    mission_exec.add_child(set_mission_complete)

    # do mission, if all else fails, idle
    mission_fb.add_child(mission_exec)
    mission_fb.add_child(idle)


    # the main meat of the tree
    mission_seq = ReactiveSeq(name='Mission')
    # make the mission
    mission_seq.add_children([safety_fb, mission_fb, idle])


    #####################
    # ROOT
    #####################

    # we want to tick all children at once, namely the topic listeners and
    # the rest of the tree
    root = pt.composites.Parallel("Root")
    # finish the tree by adding the main subtrees to the root
    root.add_children([topics2bb, mission_seq])
    root.add_children([topics2bb])



    rospy.init_node('tree')
    # a nice wrapper for visiting and such
    tree = ptr.trees.BehaviourTree(root)
    # shut down the tree when ctrl-c is received
    shutdown_tree = lambda t: t.interrupt()
    rospy.on_shutdown(functools.partial(shutdown_tree, tree))

    root.setup()
    # setup the tree
    if not tree.setup(timeout=10):
        print('TREE COULD NOT BE SETUP')
        sys.exit(1)

    #tree.setup(timeout=10)
    # show the tree's status for every tick
    tick_printer = lambda t: pt.display.print_ascii_tree(t.root, show_status=True)
    # run
    tree.tick_tock(sleep_ms=500, post_tick_handler=tick_printer)






