#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot OS project and is released under the "Apache Licence, Version 2.0".
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-02-16
# modified: 2021-10-22
#

import asyncio
import traceback
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.event import Event, Group
from core.orientation import Orientation
from core.message import Message
from core.subscriber import Subscriber
from core.macro_publisher import MacroPublisher

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MacroSubscriber(Subscriber):

    CLASS_NAME = 'macro'

    '''
    A subscriber to macro events. This receives MACRO events and queues
    the macro from the MacroPublisher's macro library for execution.

    :param config:            the application configuration
    :param message_bus:       the message bus
    :param message_factory:   the message factory, used to create a completion message
    :param macro_publisher:   the macro publisher/processor
    :param level:             the logging level
    '''
    def __init__(self, config, message_bus, message_factory, macro_publisher, level=Level.INFO):
        Subscriber.__init__(self, MacroSubscriber.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        if not isinstance(macro_publisher, MacroPublisher):
            raise ValueError('wrong type for macro_publisher argument: {}'.format(type(macro_publisher)))
        self._message_factory = message_factory
        self._macro_publisher = macro_publisher
        self.add_events([ Event.by_group(Group.MACRO), Event.AVOID ])
#       self.add_events([ Event.by_group(Group.MACRO), Event.by_group(Group.BEHAVIOUR) ])

#   async def _arbitrate_message(self, message):
#       '''
#       Pass the message on to the Arbitrator and acknowledge that it has been
#       sent (by setting a flag in the message).
#       '''
#       await self._message_bus.arbitrate(message.payload)
#       # increment sent acknowledgement count
#       self._log.info('acknowledging message {}; with payload value: {:5.2f}cm'.format(message.name, message.payload.value))
#       message.acknowledge_sent()
#       self._log.info('arbitrated message:    ' + Fore.WHITE + '{}'.format(message.name)
#               + Fore.CYAN + ' with payload for event: {}; value: {:5.2f}cm'.format(message.payload.event.name, message.payload.value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        if not isinstance(message, Message):
            raise TypeError('expected Message argument, not {}'.format(type(message)))
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected.')
        _event = message.event
        self._log.info('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.name))
        if _event == Event.MACRO:
            _name = _event.name
            self._log.info('processing MACRO message {} with name: '.format(message.name) + Fore.YELLOW + '{}'.format(_name))
            self._macro_publisher.queue_by_name(message.payload)
        elif self.acceptable(message):
            _name = _event.name
            self._log.info('processing acceptable message {} with macro name: '.format(message.name) + Fore.YELLOW + '{}'.format(_name))
            _value = message.payload.value
            self._log.info('type: {}; name {}; '.format(type(_value), _value)) # TODO react differently depending on which bumper
            # TODO permit passing a Message as a value so we don't need to create one here...
            # add a VELOCITY completion message as a payload
            _message = self._message_factory.create_message(Event.VELOCITY, _value)
            self._macro_publisher.queue_macro_by_name(_name, _message)
#           self._macro_publisher.queue_macro_by_name(_name, message.payload)
#           self._macro_publisher.queue_event(message.payload)
        else:
            self._log.warning('unrecognised {} event on message {}'.format(_event.name, message.name))
        await Subscriber.process_message(self, message)
        self._log.debug('post-processing message {}'.format(message.name))

#EOF
