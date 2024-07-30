#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-02-21
# modified: 2024-04-26
#
# currently unused.
#

import time, itertools
import datetime as dt
from colorama import init, Fore, Style
init()

from core.event import Event
from core.logger import Logger, Level
from core.controller import Controller

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GamepadController(Controller):
    '''
    A controller class for a Gamepad.
    '''
    def __init__(self, message_bus, level):
        Controller.__init__(self, message_bus, level)
        # override controller's log:
        self._log = Logger('gamepad-cntl', level)
        self._previous_event       = None # Event.NOOP
#       self._enabled              = True
#       self._event_counter        = itertools.count()
#       self._event_count          = next(self._event_counter)
#       self._state_change_counter = itertools.count()
#       self._state_change_count   = next(self._state_change_counter)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'gp-controller'

#   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   def enable(self):
#       self._enabled = True
#       self._log.info('enabled.')

#   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   def disable(self):
#       self._enabled = False
#       self._log.info('disabled.')

#   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   def get_current_message(self):
#       return self._current_message

#   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   def _clear_current_message(self):
#       self._log.debug('clear current message  {}.'.format(self._current_message))
#       self._current_message = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def callback(self, payload):
        '''
        Responds to the Event contained within the Payload.
        '''
#       self._log.debug('callback with payload {}'.format(payload.event.name))
        if not self._enabled:
            self._log.debug('action ignored: controller disabled.')
            return
        self._event_count = next(self._event_counter)
        if payload.event == self._previous_event:
            self._log.info(Fore.CYAN + 'no state change on event: ' + Style.BRIGHT + ' {}'.format(self._previous_event.name)
                    + Fore.BLACK + Style.NORMAL + '[{:d}/{:d}]'.format(self._state_change_count, self._event_count))
            return
        self._state_change_count = next(self._state_change_counter)

        _start_time = dt.datetime.now()
        _event = payload.event
        self._log.info(Fore.CYAN + 'act on event: ' + Style.BRIGHT + ' {}'.format(_event.name)
                + Fore.BLACK + Style.NORMAL + ' [{:d}/{:d}]'.format(self._state_change_count, self._event_count))

        # system events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if _event is Event.GAMEPAD:
           self._log.info('event: gamepad.')

        elif _event is Event.SHUTDOWN:
           self._log.info('event: SHUTDOWN.')

        # unrecognised event ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        else:
            self._log.error('unprocessed event: {}'.format(_event))

        self._previous_event = _event
        _delta = dt.datetime.now() - _start_time
        _elapsed_ms = int(_delta.total_seconds() * 1000)
        self._log.debug(Fore.MAGENTA + Style.DIM + 'elapsed: {}ms'.format(_elapsed_ms) + Style.DIM)

# EOF
