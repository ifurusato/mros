#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-02-21
# modified: 2021-04-26
#

import time, itertools
import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Controller(Component):
    '''
    A default controller class that receives callbacks (to the 'callback'
    method) when Events appear on the MessageBus' Arbitratror.
    '''
    def __init__(self, message_bus, level):
        self._log = Logger('controller', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._message_bus          = message_bus
        self._previous_payload     = None
        self._event_counter        = itertools.count()
        self._event_count          = next(self._event_counter)
        self._state_change_counter = itertools.count()
        self._state_change_count   = next(self._state_change_counter)
        self._message_bus.register_controller(self)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_log_level(self, level):
        self._log.level = level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'def-controller'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_current_message(self):
        return self._current_message

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _clear_current_message(self):
        self._log.debug('clear current message  {}.'.format(self._current_message))
        self._current_message = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def callback(self, payload):
        '''
        Responds to the Event contained within the Payload.
        '''
#       self._log.debug('callback with payload {}'.format(payload.event.name))
        if not self.enabled:
            self._log.warning('action ignored: controller disabled.')
            return
        else:
            return # we currently do don't anything with the callback...

        self._event_count = next(self._event_counter)

        self._previous_payload = payload
        self._state_change_count = next(self._state_change_counter)

        _start_time = dt.datetime.now()
        _event = payload.event

        _delta = dt.datetime.now() - _start_time
        _elapsed_ms = int(_delta.total_seconds() * 1000)
#       self._log.info(Fore.MAGENTA + Style.DIM + 'elapsed: {}ms'.format(_elapsed_ms) + Style.DIM)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_statistics(self):
        self._log.info('{}:'.format(self.name) + Fore.YELLOW + '\t{} events; {} state changes.'.format(self._event_count, self._state_change_count))

# EOF
