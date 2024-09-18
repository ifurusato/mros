#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-03-10
# modified: 2021-06-29
#

import asyncio
import random
import traceback
from asyncio import CancelledError
#from typing import final
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.component import Component
from core.util import Util
from core.event import Event, Group
from core.message import Message
from core.fsm import FiniteStateMachine, State
from core.message_bus import MessageBus

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Subscriber(Component, FiniteStateMachine):

    LOG_INDENT = ( ' ' * 60 ) + Fore.CYAN + ': ' + Fore.CYAN

    '''
    Extends Component and FiniteStateMachine as a subscriber to messages
    from the message bus.

    :param log_or_name:  the Logger or subscriber name (for logging)
    :param config:       the application configuration
    :param message_bus:  the message bus
    :param suppressed:   the initial state of the suppressed flag
    :param enabled:      the initial state of the enabled flag
    :param level:        the logging level
    '''
    def __init__(self, log_or_name, config, message_bus, suppressed=False, enabled=False, level=Level.INFO):
        if isinstance(log_or_name, Logger):
            self._log = log_or_name
            self._name = self._log.name
        elif isinstance(log_or_name, str):
            self._log = Logger('sub:{}'.format(log_or_name), level)
            self._name = log_or_name
        else:
            raise ValueError('wrong type for log_or_name argument: {}'.format(type(log_or_name)))
        self._id   = random.randint(10000,99999)
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(self._name)))
        self._config = config
#       if not isinstance(message_bus, MessageBus):
#           raise ValueError('wrong type for message bus argument: {}'.format(type(message_bus)))
        self._message_bus = message_bus
        if not isinstance(suppressed, bool):
            raise ValueError('wrong type for suppressed argument: {}'.format(type(suppressed)))
        if not isinstance(enabled, bool):
            raise ValueError('wrong type for enabled argument: {}'.format(type(enabled)))
        Component.__init__(self, self._log, suppressed, enabled)
        FiniteStateMachine.__init__(self, self._log, self._name)
        self._events = [] # list of acceptable event types
        self._brief  = True # brief messages by default
        self._message_bus.register_subscriber(self)
        self._permit_resend = False
#       self._log.info(Fore.BLACK + 'ready (superclass).')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def permit_resend(self):
        '''
        If called, permits messages to be resent without issuing a warning.
        '''
        self._permit_resend = True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_log_level(self, level):
        self._log.level = level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def config(self):
        return self._config

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def message_bus(self):
        return self._message_bus

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_gc(self):
        return False

    # events ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def events(self):
        '''
        A function to return the list of events that this subscriber accepts.
        '''
        return self._events

    def add_events(self, events):
        '''
        Adds the list of events to the list that this subscriber accepts.
        '''
        if not isinstance(events, list) and not isinstance(events, Group):
            raise ValueError('expected list or Group argument, not: {}'.format(type(events)))
        elif isinstance(events, Group):
            _events = Event.by_group(events)
            self.add_events(_events)
        else:
            for _event in events:
                if isinstance(_event, Event):
                    self.add_event(_event)
                elif isinstance(_event, list):
                    self.add_events(_event)
                elif isinstance(_event, Group):
                    _events = Event.by_group(_event)
                    self.add_events(_events)
                else:
                    raise ValueError('unrecognised event value: {}'.format(type(_event)))

    def add_event(self, event):
        '''
        Adds an event to the list that this subscriber accepts.
        '''
        if not isinstance(event, Event):
            raise TypeError('expected Event argument, not {}'.format(type(event)))
        self._events.append(event)
#       self._log.debug('added \'{}\' event to subscriber {} ({:d} events).'.format(event.name, self._name, len(self._events)))

    def print_events(self):
        if self._events == [Event.ANY]:
            return '[ANY]'
        elif self._events:
            _events = []
            for _event in self._events:
                if not isinstance(_event, Event):
                    raise TypeError('expected Event, not {}'.format(type(_event)))
                _description = _event.name.replace(' ','-')
                _events.append('{} '.format(_description))
            return ''.join(_events)
        else:
            return '[NONE]'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def acceptable(self, message):
        '''
        A filter that returns True if the message's event type is acceptable
        to this subcriber, either by being included in the list of acceptable
        events, or by matching the special case Event.ANY.
        '''
        if not isinstance(message, Message):
            raise TypeError('expected Message argument, not {}'.format(type(message)))
        return message.event is Event.ANY or message.event in self._events

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#   @final
    async def consume(self):
        '''
        Awaits a message on the message bus, first peeking at it from the queue,
        filtering on event type.

        Kick off various tasks to consume (process) the message by first creating
        an asyncio event. Tasks are then created to process the message followed
        by a cleanup task. If the latter is overridden it should also called
        by the subclass method as it flags the message as expired. Once these
        tasks have been created the asyncio event flag is set, indicating that
        the message has been consumed, and can subsequently garbage collected.

        This method is not meant to be overridden, except by the garbage
        collector. The process_message() method can be overridden.
        '''
        try:

#           self._log.debug('consume() called on {}.'.format(self.name))
            _peeked_message = await self._message_bus.peek_message()
            if not _peeked_message:
                raise QueueEmptyOnPeekError('peek returned none.')
            elif _peeked_message.gcd:
                raise GarbageCollectedError('{} cannot consume: message has been garbage collected. [1]'.format(self.name))
    
#           self._log.debug('consume() continuing for {}...'.format(self.name))
            _ackd = _peeked_message.acknowledged_by(self)
            if not _ackd and self.acceptable(_peeked_message):
                _event = asyncio.Event()
                self._log.debug(Fore.RED + 'begin event tracking for message:' + Fore.WHITE
                        + ' {}; event: {}'.format(_peeked_message.name, _peeked_message.event.name))
    
                # acknowledge we've seen the message
                _peeked_message.acknowledge(self)
    
                # this subscriber accepts this message and hasn't seen it before so consume and handle the message
#               self._log.debug('waiting to consume acceptable message:'
#                       + Fore.WHITE + ' {}; event: {}'.format(_peeked_message.name, _peeked_message.event.name))
    
                _message = await self._message_bus.consume_message()
                self._message_bus.consumed()
#               if self._message_bus.verbose:
#                   self._log.debug('consumed acceptable message:' + Fore.WHITE + ' {}; event: {}'.format(_message.name, _message.event.name))
    
                # handle acceptable message
                if self._message_bus.verbose:
                    _elapsed_ms = (dt.now() - _message.timestamp).total_seconds() * 1000.0
                    self._print_message_info('process message:', _message, _elapsed_ms)
#               self._log.debug('creating task for processing message:' + Fore.WHITE + ' {}; event: {}'.format(_message.name, _message.event.name))
                # create message processing task
                asyncio.create_task(self.process_message(_message), name='{}:process-message-{}'.format(self.name, _message.name))
    
                # create message cleanup task
                _cleanup_task = asyncio.create_task(self._cleanup_message(_message), name='{}:cleanup-message-{}'.format(self.name, _message.name))
                _cleanup_task.add_done_callback(self._done_callback)
    
#               breakpoint()
    
#               self._log.debug('end event tracking for message:' + Fore.WHITE + ' {}; event: {}'.format(_message.name, _message.event.name))
                _event.set()
    
                # we've handled message so pass along to arbitrator
                if _message.sent == 0:
#                   self._log.debug('sending message: {}; event: {} to arbitrator...'.format(_message.name, _message.event.name))
                    await self._arbitrate_message(_message)
#                   self._log.debug('sent message:' + Fore.WHITE + ' {}; event: {} to arbitrator.'.format(_message.name, _message.event.name))
                elif _message.sent == -1:
                    # don't arbitrate, just keep republishing this message
                    pass
                elif not self._permit_resend:
                    self._log.warning('message: {} already sent; event: {}'.format(_message.name, _message.event.name))
    
#               # keep track of timestamp of last message
#               self._log.debug('last message timestamp: {}'.format(_message.timestamp))
#               self._message_bus.last_message_timestamp = _message.timestamp
                # republish the message
#               self._log.debug('awaiting republication of message:' \
#                   + Fore.WHITE + ' {}; event: {}'.format(_message.name, _message.event.name))
                await self._message_bus.republish_message(_message)
#               self._log.debug('message:' + Fore.WHITE + ' {} with event: {}'.format(_message.name, _message.event.name) + ' has been republished.')
    
            elif not _ackd:
                # if not already ack'd, acknowledge we've seen the message
#               self._log.debug('acknowledging unacceptable message:' + Fore.WHITE + ' {}; event: {} (queue: {:d} elements)'.format(
#                       _peeked_message.name, _peeked_message.event.name, self._message_bus.queue_size))
                _peeked_message.acknowledge(self)
#           self._log.debug('consume() complete on {}.'.format(self.name))

        except Exception as e:
            self._log.error('{} thrown during consume: {}\n{}'.format(type(e), e, traceback.format_exc()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _done_callback(self, task):
        '''
        Signal the message bus callback has completed.
        '''
        self._message_bus.clear_tasks()
#       self._log.debug('callback on cleanup task complete; {}'.format(task.get_name()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message, i.e., pass its Payload along to the Arbitrator,
        which passes the highest priority Payload to the Controller to change
        the state of the robot.

        This method is meant to be overridden by subclasses. Its only
        responsibility is to set the message's processed flag.

        :param message:  the message to process.
        '''
#       self._log.debug('processing message {}'.format(message.name))
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        # indicate that this subscriber has processed the message
        message.process(self)
#       self._log.debug('processed message {}'.format(message.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _arbitrate_message(self, message):
        '''
        Pass the message on to the Arbitrator and acknowledge that it has been
        sent (by setting a flag in the message).
        '''
        await self._message_bus.arbitrate(message.payload)
        # increment sent acknowledgement count
        message.acknowledge_sent()
#       if self._message_bus.verbose:
#           self._log.info('arbitrated payload for event {}; value: {}'.format(message.payload.event.name, message.payload.value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _cleanup_message(self, message):
        '''
        Cleanup tasks related to completing work on a message (by this subscriber).

        This currently sets the message's 'expire' flag as True and tells the
        message bus to clear any completed tasks. It's not really async but is
        declared as such as part of the asyncio experiment.

        :param message:  consumed message that is done being processed.
        '''
#       if self._message_bus.verbose:
#           self._log.debug('begin cleanup of message: {}'.format(message.name))
        if message.gcd:
            self._log.warning('cannot cleanup message: message has been garbage collected. [4]')
            return
        # set message flag as expired
        self._log.debug('message {} expired by subscriber: {}.'.format(message.name, self._name))
        message.expire()
        # clear any tasks related to the message
#       self._message_bus.clear_tasks()
#       self._log.debug('end cleanup of message: {}'.format(message.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _print_message_info(self, title, message, elapsed):
        '''
        Print an informational string followed by details about the message.

        :param title:    the title string
        :param message:  the message and its payload to display
        :param elapsed:  the optional elapsed time (in milliseconds) for an operation
        '''
        if self._brief:
            self._log.info(Style.BRIGHT + title + Style.NORMAL \
                    + ' id: ' + Style.BRIGHT + '{};'.format(message.name) + Style.NORMAL \
                    + ' event: ' + Style.BRIGHT + ( '{}; '.format(message.event.name) if message.event else 'n/a' ) + Style.NORMAL \
                    + ' value: ' + Style.BRIGHT + Util.get_formatted_value(message.payload.value))
        else:
            self._log.info(Style.BRIGHT + title + Style.NORMAL + '\n' \
                    + Subscriber.LOG_INDENT + 'id: ' + Style.BRIGHT + '{};'.format(message.name) + Style.NORMAL \
                    + ' event: ' + Style.BRIGHT + ( '{}; '.format(message.event.name) if message.event else 'n/a: [gc\'d] ' ) + Style.NORMAL \
                    + ' value: ' + Style.BRIGHT + Util.get_formatted_value(message.payload.value) + '\n' + Style.NORMAL \
                    + Subscriber.LOG_INDENT + '{:d} procd;'.format(message.processed) + ' sent {:d}x;'.format(message.sent) \
                            + ' expired? {}\n'.format(self._message_bus.is_expired(message)) \
                    + Subscriber.LOG_INDENT + 'procd by:\t{}\n'.format(message.print_procd()) \
                    + Subscriber.LOG_INDENT + 'acked by:\t{}\n'.format(message.print_acks()) \
                    + Subscriber.LOG_INDENT + Util.get_formatted_time('msg age: ', message.age) + '; ' + Util.get_formatted_time('elapsed: ', elapsed))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        '''
        The necessary state machine call to start the publisher, which performs
        any initialisations of active sub-components, etc.
        This also enables the Subscriber, which is upon initialisation disabled.
        '''
        if self.state is not State.STARTED:
            self._log.debug('subscriber {} started.'.format(self.name))
            FiniteStateMachine.start(self)
            self.enable()
        else:
            self._log.debug('subscriber {} already started.'.format(self.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            Component.disable(self)
            FiniteStateMachine.disable(self)
            self._log.debug('subscriber {} disabled.'.format(self.name))
        else:
            self._log.warning('subscriber {} already disabled.'.format(self.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        if not self.closed:
            Component.close(self)
            FiniteStateMachine.close(self)
            self._log.debug('subscriber {} closed.'.format(self.name))
        else:
            self._log.warning('subscriber {} already closed.'.format(self.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __key(self):
        return (self.name, self._id)

    def __hash__(self):
        return hash(self.__key())

    def __eq__(self, obj):
        return isinstance(obj, Subscriber) and obj.name == self.name

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GarbageCollector(Subscriber):
    CLASS_NAME = 'gc'
    '''
    Extends subscriber as a garbage collector that eliminates messages after
    they've passed the publish cycle. This subscriber accepts ANY event type.

    :param name:         the subscriber name (for logging)
    :param config:       the application configuration
    :param message_bus:  the message bus
    :param level:        the logging level
    '''
    def __init__(self, config, message_bus, level=Level.INFO):
        Subscriber.__init__(self, GarbageCollector.CLASS_NAME, config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        self.add_event(Event.ANY)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_gc(self):
        return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def acceptable(self, message):
        '''
        A filter that returns True if the message is either expired and/or
        fully acknowledged.
        '''
        return self._message_bus.is_expired(message) or message.fully_acknowledged

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def consume(self):
        '''
        Overrides the method on Subscriber to first peek, and then if acceptable
        (expired and/or fully acknowledged), then consume and explicitly garbage
        collect the message.
        '''
        _peeked_message = await self._message_bus.peek_message()
        if not _peeked_message:
            raise QueueEmptyOnPeekError('peek returned none.')
        elif _peeked_message.gcd:
            self._log.warning('message has already been garbage collected. [1]'.format(self.name))
#       if self._message_bus.verbose:
#           self._log.debug('gc-consume() message:' + Fore.WHITE + ' {}; event: {}'.format(_peeked_message.name, _peeked_message.event.name))

        # garbage collect (consume) if filter accepts the peeked message
        if self.acceptable(_peeked_message):
            _message = await self._message_bus.consume_message()
            self._message_bus.consumed()
            _message.gc() # mark as garbage collected and don't republish
            if not _message.sent:
                self._log.warning('garbage collected undelivered message: {}; event {} of group {}; value: {}'.format(
                        _message.name, _message.event.name, _message.event.group.name, _message.value))
#           elif self._message_bus.verbose:
#           self._log.info('garbage collected message:' + Fore.WHITE + ' {}; event: {}'.format(_message.name, _message.event.name))
        else:
            # acknowledge we've seen the message
            _peeked_message.acknowledge(self)
#           self._log.info('acknowledged unacceptable message:' \
#                   + Fore.WHITE + ' {}; event: {} (queue: {:d} elements)'.format(
#                   _peeked_message.name, _peeked_message.event.name, self._message_bus.queue_size))

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GarbageCollectedError(Exception):
    '''
    The garbage collector refused to process the message that has already been
    garbage collected.
    '''
    pass

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class QueueEmptyOnPeekError(Exception):
    '''
    An awaited peek at the queue failed to return a message.
    '''
    pass

#EOF
