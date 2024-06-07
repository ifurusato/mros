#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-19
# modified: 2021-06-26
#

from enum import Enum

from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class State(Enum):
    NONE     = ( 0, "none")
    INITIAL  = ( 1, "initial")
    STARTED  = ( 2, "started")
    ENABLED  = ( 3, "enabled")
    DISABLED = ( 4, "disabled")
    CLOSED   = ( 5, "closed")

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name):
        self._name = name

    # this makes sure the name is read-only
    @property
    def name(self):
        return self._name

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class IllegalStateError(RuntimeError):
    pass

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class FiniteStateMachine(object):
    '''
    Implementation of a Finite State Machine (FSM).

    This requires an initial start() followed by repeated transitions
    between enable() and disable(), followed by a terminal close().
    '''
    def __init__(self, logger, task_name, level=Level.INFO):
        self._log = logger #Logger('fsm:{}'.format(task_name), level)
        self._state     = State.NONE
        self._task_name = task_name
        FiniteStateMachine.__transition__(self, State.INITIAL)
        self._log.debug('fsm initialised.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __transition__(self, next_state):
        '''
        This method provides the functionality of a transition table, throwing
        exceptions or logging warnings if the transition is either invalid or
        ill-advised (resp.).
        '''
        self._log.debug('transition in {} from {} to {}.'.format(self._task_name, self._state.name, next_state.name))
        # transition table:
        if self._state is State.NONE:
            if next_state is State.INITIAL:
                pass
            else:
                raise IllegalStateError('invalid transition in {} from {} to {} (expected INITIAL).'.format(self._task_name, self._state.name, next_state.name))
        if self._state is State.INITIAL:
            # we permit DISABLED for when we've never really got started, as a proper transition to CLOSED
            if any([ next_state is State.STARTED, next_state is State.DISABLED, next_state is State.CLOSED ]):
                pass
            else:
                raise IllegalStateError('invalid transition in {} from {} to {} (expected STARTED).'.format(self._task_name, self._state.name, next_state.name))
        elif self._state is State.STARTED:
            if any([ next_state is State.ENABLED, next_state is State.DISABLED, next_state is State.CLOSED ]):
                pass
            else:
                raise IllegalStateError('invalid transition in {} from {} to {} (expected ENABLED, DISABLED, or CLOSED).'.format(self._task_name, self._state.name, next_state.name))
        elif self._state is State.ENABLED:
            if any([ next_state is State.DISABLED, next_state is State.CLOSED ]):
                pass
            elif next_state is State.ENABLED:
                self._log.warning('suspect transition in {} from {} to {}.'.format(self._task_name, self._state.name, next_state.name))
            else:
                raise IllegalStateError('invalid transition in {} from {} to {} (expected DISABLED or CLOSED).'.format(self._task_name, self._state.name, next_state.name))
        elif self._state is State.DISABLED:
            if any([ next_state is State.ENABLED, next_state is State.CLOSED ]):
                pass
            elif next_state is State.DISABLED:
                self._log.warning('suspect transition in {} from {} to {}.'.format(self._task_name, self._state.name, next_state.name))
            else:
                raise IllegalStateError('invalid transition in {} from {} to {} (expected ENABLED or CLOSED).'.format(self._task_name, self._state.name, next_state.name))
        elif self._state is State.CLOSED:
            raise IllegalStateError('invalid transition in {} from {} to {} (already CLOSED).'.format(self._task_name, self._state.name, next_state.name))
        self._state = next_state

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def state(self):
        return self._state

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        self._log.debug('start.')
        FiniteStateMachine.__transition__(self, State.STARTED)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._log.debug('enable.')
        FiniteStateMachine.__transition__(self, State.ENABLED)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self._log.debug('disable.')
        FiniteStateMachine.__transition__(self, State.DISABLED)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self._log.debug('close.')
        FiniteStateMachine.__transition__(self, State.CLOSED)

# EOF
