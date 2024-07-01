#!/usr/bin/env python3
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-02
# modified: 2021-04-26
#

import itertools
import datetime as dt
from asyncio.queues import PriorityQueue
from colorama import init, Fore, Style
init()

from core.logger import Logger
from core.event import Event
from core.component import Component
from core.controller import Controller

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Arbitrator(Component):
    '''
    Arbitrates a stream of events from a MessageBus according to priority,
    returning to a Controller when polled the highest priority of them.
    '''
    def __init__(self, level):
        self._log = Logger('arbitrator', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._counter     = itertools.count()
        self._count       = 0
        self._queue       = PriorityQueue()
        self._controllers = []
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_log_level(self, level):
        self._log.level = level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def controllers(self):
        return self._controllers

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def register_controller(self, controller: Controller):
        '''
        Registers the Controller with the Arbitrator. When a Payload appears
        from the MessageBus its callback(Payload) method is called.
        '''
        self._controllers.append(controller)
        self._log.info('registered controller: \'{}\''.format(controller.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def count(self):
        '''
        Return the number of payloads delivered to the Arbitrator, not including
        those sent while suppressed.
        '''
        return self._count

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def arbitrate(self, payload):
        '''
        Arbitrates the addition of the payload into the priority queue.
        If suppressed the queue is cleared so that events don't accumulate.

        If the Event Group is CLOCK this will trigger the callback without
        arbitration.
        '''
        self._log.debug('arbitrating payload: {}'.format(payload.event.name))
        if self._suppressed:
            self._queue.clear()
        else:
            _start_time = dt.datetime.now()
            self._count = next(self._counter)
#           self._log.debug('[{:03d}] putting payload: \'{}\' onto queue...'.format(self._count, payload.event.name))
            if len(self._controllers) > 0:
                await self._queue.put((payload.priority, payload))
#               self._log.debug('payload \'{}\' put onto queue: {} element{}.'.format(
#                       payload.event.name, self._queue.qsize(), '' if self._queue.qsize() == 1 else 's'))
                await self.trigger_callback()
                _elapsed_ms = int((dt.datetime.now() - _start_time).total_seconds() * 1000)
#               self._log.debug('{:4.2f}ms elapsed.'.format(_elapsed_ms))
            else:
#               self._log.warning('no registered controllers: payload ignored.')
                pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def trigger_callback(self):
        self._log.debug('trigger callback.')
        _tuple = await self._queue.get()
        _payload = _tuple[1]
        for controller in self._controllers:
            controller.callback(_payload)

# EOF
