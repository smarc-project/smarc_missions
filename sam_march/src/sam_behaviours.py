# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)


import py_trees as pt


class some_mission(pt.behaviour.Behaviour):
    def __init__(self, name):
        """
        do not do long-running things here
        just initialize variables and such
        """
        super(some_mission, self).__init__(name)
        self.counter = 0

    def setup(self, *args, **kwargs):
        """
        do the hard initializing of stuff here
        ros stuff goes here if needed (should not be needed, use actions instead)
        """
        print('some mission setup')
        # this return is needed!
        return True

    def initialise(self):
        """
        stuff to do RIGHT BEFORE update is called.
        only called when the status of the behaviour is not RUNNING
        """
        self.counter = 0
        print('some mission initialise')

    def update(self):
        """
        actual tick updates here
        """
        print('some mission ticked')
        self.counter += 1
        if self.counter < 10:
            return pt.Status.RUNNING

        if self.counter == 10:
            return pt.Status.SUCCESS


    def terminate(self, new_status):
        """
        stuff to do when the behaviour is done.
        if new_status is S/F, the behaviour completed itself
        if its INVALID, it was pre-empted
        """
        print('some mission is terminated with', new_status)



