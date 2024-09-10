#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-05-19
# modified: 2024-06-03 (complete rewrite of ExternalClock)
#

import time, itertools
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.logger import Logger, Level
from core.event import Event
from core.message import Message
from core.message_factory import MessageFactory
from core.publisher import Publisher

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class ClockPublisher(Publisher):

    CLASS_NAME = 'clock'
#   _LISTENER_LOOP_NAME = '__clock_listener_loop'

    '''
    A publisher for events triggered by a callback from an external clock
    source. This simply divides the incoming ticks and publishes a singleton
    TICK message upon every n ticks.

    Messages from the Clock are published to the asynchronous message bus,
    so timing is not assured. This clock should be used to trigger events
    that aren't stricly time-critical but need to be scheduled every now
    and then.

    :param config:            the application configuration
    :param message_bus:       the asynchronous message bus
    :param message_factory:   the factory for creating messages
    :param level:             the log level
    '''
    def __init__(self, config, message_bus, message_factory, irq_clock, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        Publisher.__init__(self, ClockPublisher.CLASS_NAME, config, message_bus, message_factory, level=self._level)
        self._message_bus = message_bus
        self._message_factory = message_factory
        self._queue_publisher = globals.get('queue-publisher')
        if self._queue_publisher is None:
            raise Exception('queue publisher is not available.')
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._irq_clock    = irq_clock
        _cfg = config['mros'].get('publisher').get('clock')
        self._divider      = _cfg.get('divider')
        self._log.info('clock divider:\t{:d}'.format(self._divider))
        self._counter      = itertools.count()
        self._millis       = lambda: int(round(time.perf_counter() * 1000))
        self._timestamp = dt.now()
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Publisher.enable(self)
        if self.enabled:
#           if self._message_bus.get_task_by_name(ClockPublisher._LISTENER_LOOP_NAME):
#               self._log.warning('already enabled.')
#           else:
            self._irq_clock.add_callback(self._irq_callback_method)
        else:
            self._log.warning('failed to enable clock publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _irq_callback_method(self):
        '''
        This method is called by the IRQ clock.
        '''
        if self.enabled:
            _count = next(self._counter)
            if _count % self._divider == 0:
                _message = self._message_factory.create_message(Event.TICK, self._millis())
                self._queue_publisher.put(_message)
                _elapsed_ms = int((dt.now() - self._timestamp).total_seconds() * 1000.0)
#               self._log.info('published tick: {:d}ms elapsed.'.format(_elapsed_ms))
                self._timestamp = dt.now()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self._irq_clock.remove_callback(self._irq_callback_method)
        Publisher.close(self)
        self._log.info('closed.')

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class TickMessage(Message):
    '''
    A singleton Message.
    '''
    def __init__(self, value):
        Message.__init__(self, Event.TICK, value)

    @property
    def expired(self):
        return False

    def gc(self):
        pass

    @property
    def gcd(self):
        return False

    def acknowledge(self, subscriber):
        pass

    @property
    def fully_acknowledged(self):
        return False

    def acknowledged_by(self, subscriber):
        return False

    @property
    def sent(self):
        return -1

#EOF
