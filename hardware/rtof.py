#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-04-02
# modified: 2024-04-06
#
# Description:
#
# Populates an 8x8 matrix from data polled from a VL53L5CX 8x8 ToF sensor,
# performs some further processing, including a display to the console.
#
# For each column, returns the minimum, maximum, and mean values, as well
# as the index of the largest mean value.
#
#
# Usage:
#
# The first time the script is been executed it will upload the sensor's
# firmware, which takes quite awhile. Executions of the script may
# thereafter include a 'skip' argument to avoid the firmware upload,
# which survives until power down.
#
# Hardware-wise, this requires a VL53L5CX.
#
# Depencies/requirements include:
#
#  * vl53l5cx library: (https://github.com/pimoroni/vl53l5cx-python),
#  * numpy and colorama
#
# Library installed at:
#
#  /home/pi/.local/lib/python3.11/site-packages/vl53l5cx_ctypes
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import time, datetime

import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import RANGING_MODE_CONTINUOUS # STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE, RANGING_MODE_AUTONOMOUS
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class RangingToF(Component):
    CLASS_NAME = 'RangingToF'
    '''
    The VL53L5CX Ranging Time-of-Flight sensor provides an 8 x 8 grid of distance
    values. This publishes messages of various events. TBD

    :param config:            the YAML based application configuration
    :param skip_init:         if True the default firmware load is avoided
    :param level:             the logging Level
    '''
    def __init__(self, config, message_bus, message_factory, skip_init=False, suppressed=False, enabled=False, level=Level.INFO):
#   def __init__(self, config, skip_init=False, suppressed=False, enabled=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger("rtof", level)
        Component.__init__(self, self._log, suppressed, enabled)
        if config is None:
            raise ValueError('no configuration provided.')
        self._log.info('configuring time-of-flight sensor...')
        _config = config['mros'].get('hardware').get('rtof')
        _enabled = _config.get('enabled')
        self._log.info('enabled: {}'.format(_enabled))
        # set up VL53L5CX
        self._vl53 = self._getVL53(skip_init)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def name(self):
        return 'rtof'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _getVL53(self, skip):
        '''
        Instantiate the sensor.
        '''
        stime = datetime.datetime.now()
        if skip:
            self._log.info("initialising VL53L5CX…")
            vl53 = vl53l5cx.VL53L5CX(skip_init=True)
        else:
            self._log.info("uploading firmware to VL53L5CX, please wait…")
            vl53 = vl53l5cx.VL53L5CX()
        vl53.set_resolution(8 * 8)
        vl53.set_ranging_frequency_hz(15)
        vl53.set_integration_time_ms(20)
        vl53.set_ranging_mode(RANGING_MODE_CONTINUOUS)
        executionMs = int((datetime.datetime.now() - stime).total_seconds() * 1000)
        self._log.info(Fore.BLUE + 'get VL53: {}ms elapsed.'.format(executionMs))
        return vl53

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def getData(self):
        '''
        Wait until the data from the vl53 is ready, returning the data,
        otherwise None upon a timeout.
        '''
        stime = datetime.datetime.now()
        for i in range(1,20):
            if self._vl53.data_ready():
                _data = self._vl53.get_data() # 2d array of distance
                executionMs = int((datetime.datetime.now() - stime).total_seconds() * 1000)
                self._log.info('get data: {:d}ms elapsed.'.format(executionMs))
                return _data
            time.sleep(1 / 1000)
        return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.enabled:
            Component.enable(self)
            self._vl53.start_ranging()
            self._log.info('enabled rtof.')
        else:
            self._log.warning('already enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        if self.enabled:
            self._vl53.stop_ranging()
            Component.disable(self)
            self._log.debug('successfully disabled.')
        else:
            self._log.warning("already disabled.")

#EOF
