
from collections import defaultdict

import smach

__all__ = ['DoOnExit']


class DoOnExit(smach.state_machine.StateMachine):
    """DoOnExit Container

    This container inherits functionality from L{smach.StateMachine} extended
    with the option of adding finally states that runs if the parent container
    returns one of the specified outcomes (or all by default).
    In other words, this container is the SM equivalent of the finally clause on
    try-except blocks, useful for cleaning up the execution of the non-finally
    states.
    The parent's outcome is just passed through.
    """
    def __init__(self,
                 outcomes,
                 input_keys=[],
                 output_keys=[]):
        """Constructor.

        @type outcomes: list of string
        @param outcomes: The potential outcomes of this container.
        sequence.
        """
        smach.state_machine.StateMachine.__init__(self, outcomes, input_keys, output_keys)
        self._exit_states = defaultdict(list)

    @staticmethod
    def add_finally(label, state, run_on=None, remapping=None):
        """Add a finally state that will run if the parent container
        returns one of the outcomes listed on run_on (or all by default).
        
        @type label: string
        @param label: The label of the finally state.
        
        @param state: An instance of a class implementing the L{State} interface.
        
        @param run_on: List of outcomes that will trigger the execution of the
        added state (all by default).

        @param remapping: A dictionary mapping local ud keys to ud
        keys in the parent container.
        """
        # Get currently opened container
        self = DoOnExit._currently_opened_container()

        if run_on is None:
            self._exit_states['__ANY__'].append((label, state))
        else:
            for outcome in run_on:
                self._exit_states[outcome].append((label, state))

        # The state will just pass-through the parent's outcome, so it inherits its transitions
        state._outcomes = self.get_registered_outcomes()
        transitions = {}
        for outcome in state._outcomes:
            transitions[outcome] = outcome
        return smach.StateMachine.add(label, state, transitions, remapping)

    def execute(self, parent_ud=smach.UserData()):
        outcome = super(DoOnExit, self).execute(parent_ud)
        # run all finally states associated to the parent's outcome, and pass the outcome through
        for label, state in self._exit_states[outcome] + self._exit_states['__ANY__']:
            state.execute(smach.Remapper(self.userdata,
                                         state.get_registered_input_keys(),
                                         state.get_registered_output_keys(),
                                         self._remappings[label]))
        return outcome
