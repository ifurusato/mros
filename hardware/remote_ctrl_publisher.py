#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-05-19
# modified: 2024-05-27
#
# _Getch at bottom.
#

import itertools
import asyncio
from threading import Timer
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.message_factory import MessageFactory
from core.logger import Logger, Level
from core.event import Event
from core.publisher import Publisher

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class RemoteControlPublisher(Publisher):

    CLASS_NAME = 'rcpub'
    _LISTENER_LOOP_NAME = '__rcp_listener_loop'

    '''
    A publisher for events derived from the values of a trio of GPIO pins
    set by their connection to an ESP32 device that is receiving external
    messages over ESP32-Now. With three pins as bits, there are eight
    possible values.

    This publisher functions with the inputs mapped directly to the GPIO pins.

    :param config:            the application configuration
    :param message_bus:       the asynchronous message bus
    :param message_factory:   the factory for creating messages
    :param level:             the log level
    '''
    def __init__(self, config, message_bus, message_factory, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        Publisher.__init__(self, RemoteControlPublisher.CLASS_NAME, config, message_bus, message_factory, level=self._level)
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._counter = itertools.count()
        self._pi             = None
        self._timer          = None
        _cfg = config['mros'].get('publisher').get('remote')
        _loop_freq_hz        = _cfg.get('loop_freq_hz')
        self._publish_delay_sec = 1.0 / _loop_freq_hz
        self._clear_delay_sec = _cfg.get('clear_delay_sec')
        self._events         = [ Event.REMOTE_A, Event.REMOTE_B, Event.REMOTE_Y, Event.REMOTE_X, Event.REMOTE_D, Event.REMOTE_R, Event.REMOTE_U, Event.REMOTE_L ]
        self._initd          = False
        # pin assignments
        self._d0_pin         = _cfg.get('d0_pin')
        self._d1_pin         = _cfg.get('d1_pin')
        self._d2_pin         = _cfg.get('d2_pin')
        self._log.info('remote control pin assignments:\t' \
                + Fore.RED   + ' d0={:d};'.format(self._d0_pin) \
                + Fore.BLUE  + ' d1={:d};'.format(self._d1_pin) \
                + Fore.GREEN + ' d2={:d}'.format(self._d2_pin))
        self._last_event = Event.NOOP
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Publisher.enable(self)
        if self.enabled:
            if not self._initd:
                try:
                    self._log.info('importing pigpio…')
                    import pigpio
                    # establish pigpio interrupts for remote control pins
                    self._log.info('enabling remote control interrupts…')
                    self._pi = pigpio.pi()
                    self._log.info('importing pigpio…')
                    if not self._pi.connected:
                        raise Exception('unable to establish connection to Pi.')
                    self._pi.set_mode(gpio=self._d0_pin, mode=pigpio.INPUT) # data 0
                    self._pi.set_mode(gpio=self._d1_pin, mode=pigpio.INPUT) # data 1
                    self._pi.set_mode(gpio=self._d2_pin, mode=pigpio.INPUT) # data 2
                    self._log.info('configuration complete…')
                except Exception as e:
                    self._log.warning('error configuring bumper interrupts: {}'.format(e))
                finally:
                    self._initd = True
            if self._message_bus.get_task_by_name(RemoteControlPublisher._LISTENER_LOOP_NAME):
                self._log.warning('already enabled.')
            else:
                self._log.info('creating task for remote listener loop…')
                self._message_bus.loop.create_task(self._remote_listener_loop(lambda: self.enabled), name=RemoteControlPublisher._LISTENER_LOOP_NAME)
                self._log.info('enabled.')
        else:
            self._log.warning('failed to enable publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _remote_listener_loop(self, f_is_enabled):
        self._log.info('starting bumper listener loop.')
        # initial throwaway message
        while f_is_enabled():
            _count = next(self._counter)
            _value   = None
            # read pins for 3 digit binary number
            _index_bin = '{}{}{}'.format(self._pi.read(self._d2_pin), self._pi.read(self._d1_pin), self._pi.read(self._d0_pin))
            _index = int('{}{}{}'.format(self._pi.read(self._d2_pin), self._pi.read(self._d1_pin), self._pi.read(self._d0_pin)), 2)
            # when this starts up it will create a zero unrelated to an actual event
            _message = self.message_factory.create_message(self._events[_index], _value)
#           self._log.info(Style.DIM + 'created message: {}'.format(_message))
            if _index > 0 and _count > 0 and self._last_event.num != _message.event.num: # throw out initial messages
                _event = _message.event
                if _event.num != self._last_event.num:
#                   self._log.info('remote publish count: {}; index bin: {}; index: {}'.format(_count, _index_bin, _index))
#                   self._log.info(Style.BRIGHT + 'remote-publishing message:' + Fore.WHITE + Style.NORMAL + ' {}'.format(_message.name)
#                           + Fore.CYAN + ' event: {}; '.format(_message.event.name) + Fore.YELLOW + 'timestamp: {}'.format(_message.value))
                    await Publisher.publish(self, _message)
                    pass
                self._last_event = _event
#           else:
#               self._log.info('did not publish message: {}'.format(_message))
            await asyncio.sleep(self._publish_delay_sec)
            if self._timer:
                self._timer.cancel()
        self._log.info('remote publish loop complete.')

    def _clear_last_event(self):
        self._last_event = Event.NOOP
        if self._timer:
            self._timer.cancel()
        self._timer = None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def poll(self):
        '''
        Poll the bumper sensors, executing callbacks for each.
        '''
        self._log.info('poll…')
        _start_time = dt.now()
        print(Fore.YELLOW + "poll" + Style.RESET_ALL)
        _delta = dt.now() - _start_time
        _elapsed_ms = int(_delta.total_seconds() * 1000)
        self._log.info(Fore.BLACK + '[{:04d}] poll end; elapsed processing time: {:d}ms'.format(self._count, _elapsed_ms))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable this publisher.
        '''
        Publisher.disable(self)
        self._log.info('disabled publisher.')

#EOF
