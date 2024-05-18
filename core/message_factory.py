#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2021 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2020-03-12
#

from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.message import Message
from core.message_bus import MessageBus
from core.event import Event

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MessageFactory(object):
    '''
    A factory for Messages.
    '''
    def __init__(self, message_bus, level=Level.INFO):
        self._log = Logger("msgfactory", level)
        if message_bus is None:
            raise ValueError('null message bus argument.')
#       elif not isinstance(message_bus, MessageBus):
#           raise ValueError('wrong type for message bus: {}'.format(type(message_bus)))
        self._message_bus = message_bus
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def create_message(self, event, value=None):
        '''
        Create and return a new message with the supplied event and optional
        value. Not all event types are associated with a value.
        '''
        _message = Message(event=event, value=value)
        _message.set_subscribers(self._message_bus.subscribers)
        return _message

#EOF
