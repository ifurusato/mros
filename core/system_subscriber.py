#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2021 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-19
# modified: 2021-08-20
#

from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.event import Event, Group
from core.subscriber import Subscriber, GarbageCollectedError

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SystemSubscriber(Subscriber):
    '''
    A subscriber to system events. This is required for shutdown and reacting
    to dire events.

    :param config:       the application configuration
    :param message_bus:  the message bus
    :param level:        the logging level
    '''
    def __init__(self, config, kros, message_bus, level=Level.INFO):
        Subscriber.__init__(self, 'system', config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        self._kros = kros
        self._message_bus = message_bus
        # exit KROS on dire systems event?
        self._exit_on_dire_event = config['kros'].get('exit_on_dire_event')
        self.add_events(Event.by_group(Group.SYSTEM))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _arbitrate_message(self, message):
        '''
        Pass the message on to the Arbitrator and acknowledge that it has been
        sent (by setting a flag in the message).
        '''
        await self._message_bus.arbitrate(message.payload)
        # increment sent acknowledgement count
        message.acknowledge_sent()
        self._log.info('arbitrated payload for event {}; value: {}'.format(message.payload.event.name, message.payload.value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        _event = message.event
        self._log.info('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.label))
        if Event.is_system_event(_event):
#           self._log.debug('processing system message {}'.format(message.name))
            self.dispatch_system_event(message.payload)
        else:
            self._log.warning('unrecognised event on message {}'.format(message.name) + ''.format(message.event.label))
        await Subscriber.process_message(self, message)
        self._log.debug('post-processing message {}'.format(message.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def dispatch_system_event(self, payload):
        '''
        Process an incoming event's payload.
        '''
        self._log.info('processing payload event {}'.format(payload.event.label))
        if payload.event is Event.SHUTDOWN:
            self._log.info('shut down requested.')
            self._kros.shutdown()
        elif payload.event is Event.BATTERY_LOW:
            self._log.critical('battery voltage low!')
            if self._exit_on_dire_event:
                self._log.critical('shutting down KROS...')
                self._kros.shutdown()
            else:
                self._log.critical('WARNING! WARNING! WARNING! battery voltage low! Time to shut down KROS.')
            pass
        elif payload.event is Event.HIGH_TEMPERATURE:
            self._log.critical('high temperature encoutered!')
            if self._exit_on_dire_event:
#               self._message_bus.disable()
#               self._kros.disable()
                pass # TODO
            else:
                self._log.critical('WARNING! WARNING! WARNING! high temperature encountered! Time to go into idle mode.')
            pass
        elif payload.event is Event.COLLISION_DETECT:
            self._log.critical('collision detection!')
            if self._exit_on_dire_event:
#               self._message_bus.disable()
#               self._kros.disable()
                pass # TODO
            else:
                self._log.critical('WARNING! WARNING! WARNING! collision detection! Stop everything now.')
            pass
        else:
            raise ValueError('unrecognised system event: {}'.format(payload.event.name))

#EOF
