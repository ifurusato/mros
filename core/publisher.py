#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2021-03-17
#

import asyncio
import random
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.fsm import FiniteStateMachine
from core.message_bus import MessageBus
from core.message_factory import MessageFactory

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Publisher(Component, FiniteStateMachine):
    '''
    Extends Component and FiniteStateMachine as a message/event publisher to
    the message bus.

    :param name:             the unique name for the publisher
    :param config:           the application configuration
    :param message_bus:      the asynchronous message bus
    :param message_factory:  the factory for messages
    :param suppressed:       the supprsssed flag (optional, default False)
    :param level:            the logging level
    '''
    def __init__(self, name, config, message_bus, message_factory, suppressed=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        self._log = Logger('pub:{}'.format(name), level)
        if not isinstance(name, str):
            raise ValueError('wrong type for name argument: {}'.format(type(name)))
        self._name = name
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        self._config = config
        if not isinstance(message_bus, MessageBus):
            raise ValueError('unrecognised message bus argument: {}'.format(type(message_bus)))
        self._message_bus = message_bus
        if not isinstance(message_factory, MessageFactory):
            raise ValueError('unrecognised message factory argument: {}'.format(type(message_bus)))
        self._message_factory = message_factory
        Component.__init__(self, self._log, suppressed=suppressed, enabled=False)
        FiniteStateMachine.__init__(self, self._log, name)
        self._name = name
        self._message_bus.register_publisher(self)
#       self._log.info(Fore.BLACK + 'ready (superclass).')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def logger(self):
        return self._log

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_log_level(self, level):
        self._log.level = level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def message_bus(self):
        return self._message_bus

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def message_factory(self):
        return self._message_factory

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def publish(self, message):
        '''
        Asynchronously publishes the message to the message bus.
        This is preferred to calling the message bus directly, and
        as a rule should not be overridden by subclasses.
        '''
        await self._message_bus.publish_message(message)
        # the following isn't necessary as we expect calling methods to do this for us
#       await asyncio.sleep(0.05) 

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        '''
        The necessary state machine call to start the publisher, which performs
        any initialisations of active sub-components, etc.
        '''
        FiniteStateMachine.start(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __eq__(self, obj):
        return isinstance(obj, Publisher) and obj.name == self.name

#EOF
