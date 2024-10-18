#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-03-10
# modified: 2024-06-03
#
# An asyncio-based publish/subscribe-style message bus guaranteeing exactly-once
# delivery for each message. This is done by populating each message with the
# list of subscribers. Each message is subsequently flagged as acknowledged by
# subscribers as they peek and/or consume the message.
#
# Delivery guarantees:
#
#  * At-most-once delivery. This means that a message will never be delivered
#    more than once but messages might be lost.
#  * At-least-once delivery. This means that we'll never lose a message but a
#    message might end up being delivered to a consumer more than once.
#  * Exactly-once delivery. The holy grail of messaging. All messages will be
#    delivered exactly one time.
#
# see; /usr/local/lib/python3.8/asyncio/queues.py
#
# class MessageRoutingError at bottom.
#

import sys, time, traceback, logging
import asyncio, signal
from asyncio.queues import Queue, QueueEmpty
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.util import Util
from core.component import Component
from core.event import Event
from core.message import Message
from core.arbitrator import Arbitrator
from core.numbers import Numbers

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MessageBus(Component):
    '''
    An asyncio-based asynchronous message bus.

    :param config:  the application configuration
    :param level:   the optional log level
    '''
    def __init__(self, config, level):
        self._log = Logger("bus", level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        if isinstance(config, dict):
            self._config = config
        else:
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        if level is Level.DEBUG:
            self._log.debug('logging message bus set to debug level.')
            logging.basicConfig(level=logging.DEBUG)
        self._queue = PeekableQueue(level)
        self._arbitrator = Arbitrator(level)
        _cfg = config['mros'].get('message_bus')
        self._max_age_ms             = _cfg.get('max_age_ms') # was: 20.0ms
        self._publish_delay_sec      = _cfg.get('publish_delay_sec') # was: 0.01 sec
        self._publishers             = []
        self._subscribers            = []
        self._start_callbacks        = []
        self._loop                   = None
        self._last_message_timestamp = None
        self._clip_event_list        = _cfg.get('clip_event_list') # used for printing only
        self._clip_length            = _cfg.get('clip_length')
        self._closing                = False # used during shutdown
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add_callback_on_start(self, callback):
        '''
        Add a callback to be executed upon start of the message bus.
        '''
        self._start_callbacks.append(callback)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def last_message_timestamp(self):
        '''
        Return the timestamp of the moment the last message passed through
        the message bus. Note that this is not the timestamp of the message
        itself. If no messages has passed through the bus the initial value
        is None.
        '''
        return self._last_message_timestamp

    def update_last_message_timestamp(self):
        '''
        Updates the last_message_timestamp value to dt.now().
        '''
        self._last_message_timestamp = dt.now()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def loop(self):
        '''
        Low level API, do not use.
        '''
        return self._loop

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_all_tasks(self, include_hidden=False):
        '''
        Returns the task list, not including the current task or those whose
        name starts with '__'.

        :param include_hidden:  if True return all; do not filter on task name.
        '''
        _tasks = []
        if self.loop.is_running():
            try:
                for _task in asyncio.all_tasks(loop=self._loop):
                    if _task is not asyncio.current_task() and ( include_hidden or not _task.get_name().startswith('__')):
                        _tasks.append(_task)
            except KeyboardInterrupt:
                self._log.error('Ctrl-C caught; exiting…')
            except RuntimeError as e:
                self._log.debug('cannot get task list: {}'.format(e))
        return _tasks

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear_tasks(self):
        '''
        Clears the task list of any completed tasks.
        '''
        _tasks = self.get_all_tasks()
        if len(_tasks) > 0:
            self._log.info('clearing {:d} outstanding tasks.'.format(len(_tasks)))
            for _task in _tasks:
                _task_name = _task.get_name()
                if 'shutdown' in _task_name:
                    self._log.info('ignored call to cancel shutdown task.')
                else:
                    try:
                        if not _task.cancelled():
                            _task.cancel()
                        if _task.done():
                            _tasks.remove(_task)
                        else:
                            if 'cleanup' in _task_name:
                                _tasks.remove(_task)
                                self._log.info('removed cleanup task.')
                            elif 'republish' in _task_name:
                                _tasks.remove(_task)
                                self._log.info('removed republication task.')
                            else:
                                _tasks.remove(_task)
                                if not self.closing:
                                    self._log.warning('removed unfinished task: {}'.format(_task))
                    except CancelledError:
                        self._log.warning('cancelled error: ignored.')
                    except Exception as e:
                        self._log.error('{} thrown cancelling task: {}'.format(type(e),e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_running(self):
        '''
        Returns True if the event loop is running.
        '''
        return self.loop.is_running() if self.loop else False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def queue(self):
        '''
        Returns the backing message queue.

        IMPORTANT: This is an admin method and should not be considered part of the API.
        '''
        return self._queue

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def queue_empty(self):
        '''
        Returns True if the queue is empty.

        IMPORTANT: This is an admin method and should not be considered part of the API.
        '''
        return self._queue.empty

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def pop_queue(self):
        '''
        Pops the queue but does nothing with the message, if there is one.

        IMPORTANT: This is an admin method and should not be considered part of the API.
        '''
        if not self._queue.empty:
            _message = await self._queue.get()
            self._queue.task_done()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def arbitrate(self, payload):
        await self._arbitrator.arbitrate(payload)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def queue_size(self):
        return self._queue.qsize()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear_queue(self):
        '''
        Clear the message bus of any messages.
        '''
        self._queue.clear()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def verbose(self):
        '''
        Returns True if the logger level is DEBUG.
        '''
        return self._log.level == Level.DEBUG

    @verbose.setter
    def verbose(self, verbose):
        '''
        Sets the logger level to INFO when verbose, ERROR when not verbose.
        '''
        if verbose:
            self._log.level = Level.INFO
        else:
            self._log.level = Level.ERROR
        # now set the log level of publishers and subscribers to match that of the message bus
        for publisher in self._publishers:
            publisher.set_log_level(self._log.level)
        for subscriber in self._subscribers:
            subscriber.set_log_level(self._log.level)
        self._arbitrator.set_log_level(self._log.level)
        for _controller in self._arbitrator.controllers:
            _controller.set_log_level(self._log.level)

    # publicher ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def register_publisher(self, publisher):
        '''
        Register a message publisher with the message bus.
        '''
        if publisher in self._publishers:
            raise ValueError('publisher list already contains \'{}\''.format(publisher.name))
        self._publishers.append(publisher)
        self._log.info('🥃 registered publisher: \'{}\'; {:d} publisher{} in list.'.format( \
                publisher.name, len(self._publishers), 's' if len(self._publishers) > 1 else ''))

    def get_publisher(self, name):
        '''
        Return a registered publisher by name, None if not found.
        '''
        for publisher in self._publishers:
            if publisher.name == name:
                return publisher
        return None

    def print_publishers(self):
        '''
        Print the message publishers that have been registered with the message bus.
        '''
        if not self._publishers:
            self._log.info('no registered publishers.')
            return
        self._log.info('{} publisher{}:'.format(Numbers.from_number(len(self._publishers)), 's' if len(self._publishers) > 1 else ''))
        for publisher in self._publishers:
            self._log.info(Fore.YELLOW + '\t{}'.format(publisher.name) \
                    + Fore.CYAN + ' {}enabled: '.format((' ' * max(0, (10 - len(publisher.name)))))
                    + Fore.YELLOW + '{}\t'.format(publisher.enabled)
                    + Fore.CYAN + 'suppressed: '
                    + Fore.YELLOW + '{}'.format(publisher.suppressed))

    def print_arbitrator_info(self):
        '''
        Print the stats available from the arbitrator.
        '''
        self._log.info('arbitrator:' + Fore.YELLOW + '\t{} payloads.'.format(self._arbitrator.count))
        for _controller in self._arbitrator.controllers:
            _controller.print_statistics()

    @property
    def publishers(self):
        return self._publishers

    @property
    def publisher_count(self):
        return len(self._publishers)

    # subscriber ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def register_subscriber(self, subscriber):
        '''
        Register a message subscriber with the message bus.

        Throws a ValueError if the subscriber has already been registered.
        '''
        if subscriber in self._subscribers:
            raise ValueError('subscriber list already contains \'{}\''.format(subscriber.name))
        self._subscribers.insert(0, subscriber)
        self._log.debug('registered subscriber: \'{}\'; {:d} subscriber{} in list.'.format( \
                subscriber.name, len(self._subscribers), 's' if len(self._subscribers) > 1 else ''))

    def get_subscriber(self, name):
        '''
        Return a registered subscriber by name, None if not found.
        '''
        for subscriber in self._subscribers:
            if subscriber.name == name:
                return subscriber
        return None

    def print_subscribers(self):
        '''
        Print the message subscribers that have been registered with the message bus.
        '''
        if not self._subscribers:
            self._log.info('no registered subscribers.')
            return
        self._log.info('{} subscriber{}:'.format(Numbers.from_number(len(self._subscribers)), 's' if len(self._subscribers) > 1 else ''))
        for subscriber in self._subscribers:
            if self._clip_event_list:
                _event_list = Util.ellipsis(subscriber.print_events(), self._clip_length)
            else:
                _event_list = subscriber.print_events()
            self._log.info(Fore.YELLOW + '\t{}'.format(subscriber.name)
                    + Fore.CYAN + ' {}enabled: '.format((' ' * max(0, (10 - len(subscriber.name)))))
                    + Fore.YELLOW + '{}\t'.format(subscriber.enabled)
                    + Fore.CYAN + 'suppressed: '
                    + Fore.YELLOW + '{}\t'.format(subscriber.suppressed)
                    + Fore.CYAN + 'listening for: '
                    + Fore.YELLOW + '{}'.format(_event_list))

    @property
    def subscribers(self):
        return self._subscribers

    @property
    def subscriber_count(self):
        return len(self._subscribers)

    # controller ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def register_controller(self, controller):
        '''
        Register a controller with the Arbitrator.
        '''
        self._arbitrator.register_controller(controller)
        self._log.info('registered controller: {}'.format(controller.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_expired(self, message):
        '''
        Returns True if the message has been manually expired or its age has
        passed the maximum age limit.
        '''
        return message.expired or message.age > self._max_age_ms

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _start_consuming(self):
        '''
        Enable the publishers and then start the subscribers' consume cycle.

        This is the main event loop and remains active until the message bus
        is disabled. If there are no registered subscribers the subscribe
        loop will not be entered.
        '''
        self._enable_publishers()
        self._log.info('starting {:d} subscriber{}…'.format(len(self._subscribers), '' if len(self._subscribers) == 1 else 's'))
        for subscriber in self._subscribers:
            self._log.info('starting {}…'.format(subscriber.name))
            subscriber.start()
        self._log.info('starting consume loop with {:d} subscriber{}…'.format(
                len(self._subscribers), '' if len(self._subscribers) == 1 else 's'))
        self._log.info('start callbacks…')
        for _callback in self._start_callbacks:
            _callback()
        try:
            while self.enabled and len(self._subscribers) > 0:
                for subscriber in self._subscribers:
                    await subscriber.consume()
            self._log.info('completed consume loop.')
        except KeyboardInterrupt:
            self._log.error('Ctrl-C caught; exiting…')
        finally:
            self._log.info('finally: completed consume loop.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _enable_publishers(self):
        self._log.info('enabling {:d} publisher{}…'.format(len(self._publishers), '' if len(self._publishers) == 1 else 's'))
        for publisher in self._publishers:
            publisher.start()
            if not publisher.enabled:
                publisher.enable()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_system_status(self):
        '''
        Prints the current system status to the console.
        '''
        self.print_task_info()
        self.print_arbitrator_info()
        self.print_publishers()
        self.print_subscribers()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_task_info(self):
        self._log.info('in queue:    \t' + Fore.YELLOW + '{:d} message{}.'.format(self._queue.qsize(), '' if self._queue.qsize() == 1 else 's'))
        _tasks = self.get_all_tasks()
        if len(_tasks) == 0:
            self._log.info('active tasks:\t' + Fore.YELLOW + 'none.')
        else:
            if len(_tasks) == 1:
                self._log.info('active tasks:\t' + Fore.YELLOW + '1 remains:')
            else:
                self._log.info('active tasks:\t' + Fore.YELLOW + '{:d} remain:'.format(len(_tasks)))
            for _task in _tasks:
                self._log.info(Fore.YELLOW + '    \t\t{};  \t'.format(_task.get_name()) + Fore.BLACK + ' done? {}'.format(_task.done()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def peek_message(self):
        '''
        Asynchronously waits until it peeks a message from the queue. This
        does not remove the message from the queue.
        '''
        return await self._queue.peek()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def consume_message(self):
        '''
        Asynchronously waits until it pops a message from the queue.

        NOTE: calls to this function should be await'd, and every call should
        correspond with a subsequent call to consumed().
        '''
        return self._queue.get()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def consumed(self):
        '''
        Every call to consume_message() should correspond with a call to consumed().
        This calls the asyncio.queue.task_done() method.
        '''
        self._queue.task_done()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def publish_message(self, message):
        '''
        Asynchronously publishes the Message to the MessageBus, and therefore to any Subscribers.

        NOTE: calls to this function should be await'd.
        '''
        _publish_task = asyncio.create_task(self._queue.put(message), name='publish-message-{}'.format(message.name))
        # the first time the message is published we update the 'last_message_timestamp'
        self.update_last_message_timestamp()
        await asyncio.sleep(self._publish_delay_sec)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def republish_message(self, message):
        '''
        Asynchronously re-publishes a Message to the MessageBus, making it
        available again to any Subscribers.

        NOTE: calls to this function should be await'd.
        '''
        asyncio.create_task(self._queue.put(message), name='republish-message-{}'.format(message.name))
        # when the message is republished we also update the 'last_message_timestamp'
        self.update_last_message_timestamp()

    # exception handling ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _handle_exception(self, loop, context):
        # context["message"] will always be there; but context["exception"] may not
        _exception = context.get('exception', context['message'])
        if _exception != None:
            _type = type(_exception)
            if isinstance(_exception, SystemExit):
                _code = _exception.code
                if _code == 0:
                    self._log.info('system exit.')
                else:
                    self._log.warning('caught system exit with code {}.'.format(_code))
            else:
                self._log.error('handle {} exception on loop: {} with context: {}\n{}'.format(type(_exception), loop, context, traceback.format_exc()))
        else:
            self._log.error('handling error: {}'.format(context.get('message')))
        if loop.is_running() and not loop.is_closed():
            asyncio.create_task(self.shutdown(loop), name='shutdown-on-exception')
        elif not isinstance(_exception, SystemExit):
            self._log.warning("message bus already shut down: {}".format(_exception))

    # shutdown ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    async def shutdown(self, signal=None):
        '''
        Cleanup tasks tied to the service's shutdown.
        '''
        self._log.info('starting shutdown procedure…')
        if signal:
            self._log.info('received exit signal {}…'.format(signal))
        self._log.info('nacking outstanding tasks…')
        tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
        [task.cancel() for task in tasks]
        self._log.info('cancelling {:d} outstanding tasks…'.format(len(tasks)))
        _gathered_tasks = await asyncio.gather(*tasks, return_exceptions=False)
        self._log.info('gathered tasks: {}'.format(_gathered_tasks))
        if self.loop.is_running():
            self._log.info('stopping event loop…')
            self.loop.stop()
            self._log.info('event loop stopped.')
        self._log.info('shutting down…')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __close_message_bus(self):
        '''
        This method is only to be called by the close method, used when
        shutting down services.

        This returns a value only in order to force currency.
        '''
        try:
            if self.loop:
                if self.loop.is_running():
                    self._log.info('stopping event loop…')
                    self.loop.stop()
                    self._log.info('event loop stopped.')
                if not self.loop.is_running() and not self.loop.is_closed():
                    self._log.info('closing event loop…')
                    self.loop.close()
                    self._log.info('event loop closed.')
            else:
                self._log.warning('no message bus event loop!')
        except KeyboardInterrupt:
            self._log.error('Ctrl-C caught; exiting…')
        except Exception as e:
            self._log.error('error stopping event loop: {}'.format(e))
        return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_task_by_name(self, name):
        '''
        A convenience method that returns the first found instance of a task
        with the given name, otherwise null (None).
        '''
        if name is None:
            raise ValueError('null name argument.')
        try:
            for _task in asyncio.all_tasks():
                if _task.get_name() == name:
                    return _task
        except KeyboardInterrupt:
            self._log.error('Ctrl-C caught; exiting…')
        except RuntimeError as e:
            self._log.error('unable to get task: {}'.format(e))
        return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.closed and not self.enabled:
            Component.enable(self)
            self._log.info('starting message bus forever loop…')
            # this call will block
            return self._get_event_loop()
        return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def has_event_loop(self):
        '''
        Returns True if there is an even tloop and it is running.
        '''
        return self._loop and self._loop.is_running()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_event_loop(self):
        '''
        Return the asyncio event loop, starting it if it is not already running.

        Calling this method will basically start the OS, blocking until disabled.
        '''
        if not self._loop:
            self._loop = asyncio.get_event_loop()
            if self._log.level is Level.DEBUG:
                self._loop.set_debug(True) # also set asyncio debug
            # may want to catch other signals too
            signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)
            for s in signals:
                self._loop.add_signal_handler(
                    s, lambda s = s: asyncio.create_task(self.shutdown(s), name='shutdown'),)
            self._loop.set_exception_handler(self._handle_exception)
            self._loop.create_task(self._start_consuming(), name='__event_loop__')
        if not self._loop.is_running():
            self._log.info('starting asyncio task loop…')
            self._loop.run_forever()
        return self._loop

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _kill(self):
        print('CLOSE')
        self.close()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        This disables and closes all publishers and subscribers as there is
        no expectation to be able to re-enable the message bus once disabled.
        This also simplifies the shutdown sequence.
        '''
        if self.closed:
            self._log.warning('already closed.')
        elif not self.enabled:
            self._log.warning('already disabled.')
        else:
            Component.disable(self)
            self._log.info('disabling…')
            self._log.info('closing {:d} publishers…'.format(len(self._publishers)))
#           [publisher.close() for publisher in self._publishers]
            for _publisher in self._publishers:
                self._log.info(Style.DIM + "closing publisher '{}'".format(_publisher.name))
                _publisher.close()
            self._publishers.clear()
            self._log.info('closing {:d} subscribers…'.format(len(self._subscribers)))
#           [subscriber.close() for subscriber in self._subscribers]
            for _subscriber in self._subscribers:
                self._log.info(Style.DIM + "closing subscriber '{}'".format(_subscriber.name))
                _subscriber.close()
            self._subscribers.clear()
            self.clear_tasks()
            self.clear_queue()
            _nil = self.__close_message_bus()
            time.sleep(0.1)
            self._log.info('disabled: {}'.format(_nil))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def closing(self):
        return self._closing

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        This defers most of the shutdown process to disable().
        '''
        if self.closed:
            self._log.warning('already closed.')
        elif self.closing:
            self._log.warning('already closing.')
        else:
            self._log.info('closing…')
            self._closing = True
            Component.close(self) # will call disable()
            self._closing = False
            self._log.info('closed.')

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class PeekableQueue(Queue):
    '''
    Extends the asyncio Queue to add peek() and clear() methods.
    '''
    def __init__(self, level=Level.INFO):
        Queue.__init__(self, maxsize=0)
        self._log = Logger("queue", level)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def peek(self):
        '''
        Returns the message at the top of the queue without removing it.
        If the queue is empty this returns None.

        This actually gets (removes) the message but immediately puts it
        back onto the queue.
        '''
        _message = await self.get()
        self.task_done()
        self.put_nowait(_message)
        return _message

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear(self):
        '''
        Clears the queue of any messages, brute-force, without waiting.
        '''
        while not self.empty():
            try:
                self.get_nowait()
            except Empty or QueueEmpty:
                continue
        self._log.info('cleared.')

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class MessageRoutingError(Exception):
    '''
    A Message was routed to the wrong Subscriber.
    '''
    pass

#EOF
