#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2020-01-18
# modified: 2024-05-25
#
# An integrated sensor array, including front bumper, oblique (front-facing)
# IR distance sensors, and wheel-based IR proximity sensors. Rather than
# maintain the artificial distinction between event source and publisher (as
# in the past), this class is also directly publishes events onto the message
# bus.
#
# Wraps the functionality of a Pimoroni IO Expander Breakout board, providing
# access to the values of the board's pins, which outputs 0-255 values for
# analog pins, and a 0 or 1 for digital pins.
#

import sys
import asyncio
from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level
from core.event import Event, Group
from core.stringbuilder import StringBuilder
from core.util import Util
from core.publisher import Publisher
from hardware.i2c_scanner import I2CScanner
from hardware.ina260_sensor import Ina260

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SystemPublisher(Publisher):
    CLASS_NAME = 'sys'
    _LISTENER_LOOP_NAME = '__system_publisher_loop'
    '''
    Captures system current, battery voltage and Raspberry Pi temperature
    as potential system level events.

    :param config:            the application configuration
    :param message_bus:       the asynchronous message bus
    :param message_factory:   the factory for creating messages
    :param level:             the log level
    '''
    def __init__(self, config, message_bus, message_factory, level=Level.INFO):
        Publisher.__init__(self, SystemPublisher.CLASS_NAME, config, message_bus, message_factory, level=level)
        if config is None:
            raise ValueError('no configuration provided.')
#       self._queue_publisher = queue_publisher
        _cfg = config['mros'].get('publisher').get('system')
        # config ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._publish_delay_sec = _cfg.get('publish_delay_sec')
        self._current_threshold = _cfg.get('current_threshold')
        self._voltage_threshold = _cfg.get('voltage_threshold')
        self._log.info('thresholds set for current: {:3.2f}A; voltage: {:3.2f}V.')
        self._ina260 = Ina260(config, level=self._level)
        self._log.info('ready.')

   # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'system-pub'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Publisher.enable(self)
        if self.enabled:
            if self._message_bus.get_task_by_name(SystemPublisher._LISTENER_LOOP_NAME):
                self._log.warning('already enabled.')
            else:
                self._log.info('creating task for system monitor loop…')
                self._message_bus.loop.create_task(self._monitor_loop(lambda: self.enabled), name=SystemPublisher._LISTENER_LOOP_NAME)
            self._log.info('enabled.')
        else:
            self._log.warning('failed to enable publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable this publisher.
        '''
        Publisher.disable(self)
        self._log.info('disabled publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _monitor_loop(self, f_is_enabled):

        self._log.info('monitoring IO Expander pins…')

        while f_is_enabled():

            self._log.debug('system monitor loop…')

            _voltage = self._ina260.voltage
            _current = self._ina260.current
            _power   = self._ina260.power
            self._log.info('' + Fore.MAGENTA + 'battery voltage: {:4.2f}V; '.format(_voltage)
                    + Fore.YELLOW  + 'current: {:4.2f}A; '.format(_current)
                    + Fore.GREEN   + 'power: {:4.2f}W'.format(_power))

            await asyncio.sleep(self._publish_delay_sec)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        Publisher.disable(self)
        self._log.info('disabled publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        Publisher.close(self)
        self._log.info('closed publisher.')

# EOF
