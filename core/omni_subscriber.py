#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-11-06
# modified: 2021-11-06
#

import asyncio
from colorama import init, Fore, Style
init(autoreset=True)

from core.logger import Logger, Level
from core.event import Event, Group
from core.subscriber import Subscriber

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class OmniSubscriber(Subscriber):
    CLASS_NAME = 'omni'
    '''
    A subscriber to all events.

    :param config:       the application configuration
    :param message_bus:  the message bus
    :param level:        the logging level
    '''
    def __init__(self, config, message_bus, level=Level.INFO):
        Subscriber.__init__(self, OmniSubscriber.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
#       self.add_event(Event.RGB)
        self.add_events([ Group.SYSTEM,
                Group.MACRO,
                Group.GAMEPAD,
                Group.STOP,
                Group.BUMPER,
                Group.INFRARED,
                Group.VELOCITY,
                Group.THETA,
                Group.CHADBURN,
                Group.BEHAVIOUR
#               Group.EXPERIMENT,
#               Group.OTHER
            ]) # do not include Group.CLOCK
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _arbitrate_message(self, message):
        '''
        Pass the message on to the Arbitrator and acknowledge that it has been
        sent (by setting a flag in the message).
        '''
        await self._message_bus.arbitrate(message.payload)
        message.acknowledge_sent()
        _value = message.payload.value
        self._log.info('🐹 arbitrated message ' + Fore.WHITE + '{} '.format(message.name)
                + Fore.CYAN + 'for event \'{}\' with value type: '.format(message.event.name)
                + Fore.YELLOW + '{}'.format(type(_value)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected.')
        _event = message.event
        self._log.info('🐹 pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
        await Subscriber.process_message(self, message)
        self._log.info('🐹 post-processing message {}'.format(message.name))

#EOF
