#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-28
# modified: 2024-09-08
#
# This uses 2 pins of an IO Expander interface to interface to control three
# channels of the Tiny FX LED controller, using the Tiny FX's SDA and SCL pins
# as GPIO. The two bits are treated additively as BCD, with a '00' turning off
# the channels.
#
# On the Tiny FX, treat the outputs as regular 5V and choose a resistor value
# according to the LED’s forward voltage. The LED accessories for TinyFX have
# resistors ranging from 75Ω to 220Ω. Each output is actually able to handle
# up to 600mA, but with that board's overall 1A current limit you would only
# be able to achieve it on one channel. The 'ultra-bright' white LEDs for the
# the headlights use 100Ω dropping resistors.

import time
import ioexpander as io

from core.component import Component
from core.logger import Logger, Level

class TinyFxController(Component):
    HEADLIGHT = 1
    DOWNLIGHT = 2
    RUNNING   = 3
    '''
    This uses two pins on the IO Expander to control two of the LED
    channels on the Tiny FX by repurposing its SDA and SCL pins as
    GPIO pins.

    TODO implement enable/disable/suppress/release

    :param:  config    the application configuration
    :param:  level     the log level
    '''
    def __init__(self, config, level=Level.INFO):
        if config is None:
            raise ValueError('null config argument.')
        self._log = Logger('tinyfx', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        _cfg = config['mros'].get('hardware').get('tinyfx-controller')
        self._pin1 = _cfg.get('pin_1')
        self._pin2 = _cfg.get('pin_2')
        _i2c_address = _cfg.get('i2c_address') # aft 0x18
        self._ioe = io.IOE(i2c_addr=_i2c_address)
        self._ioe.set_adc_vref(3.3)  # input voltage of IO Expander, this is 3.3 on Breakout Garden
        self._ioe.set_mode(self._pin1, io.OUT)
        self._ioe.output(self._pin1, io.HIGH)
        self._ioe.set_mode(self._pin2, io.OUT)
        self._ioe.output(self._pin2, io.HIGH)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def channel_on(self, channel_id):
        if channel_id == TinyFxController.HEADLIGHT:
            self._log.info('headlight on.')
            self._ioe.output(self._pin1, io.LOW)
            self._ioe.output(self._pin2, io.LOW)
        elif channel_id == TinyFxController.DOWNLIGHT:
            self._log.info('downlights on.')
            self._ioe.output(self._pin1, io.LOW)
            self._ioe.output(self._pin2, io.HIGH)
        elif channel_id == TinyFxController.RUNNING:
            self._log.info('running lights on.')
            self._ioe.output(self._pin1, io.HIGH)
            self._ioe.output(self._pin2, io.LOW)
        else:
            raise Exception('expected channel 1, 2 or 3, not {}.'.format(channel_id))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def on(self):
        '''
        Turn off all channels.
        '''
        self.channel_on(TinyFxController.HEADLIGHT)
        time.sleep(0.1)
        self.channel_on(TinyFxController.DOWNLIGHT)
        time.sleep(0.1)
        self.channel_on(TinyFxController.RUNNING)
        self._log.info('lights on.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def off(self):
        '''
        Turn off all channels.
        '''
        self._ioe.output(self._pin1, io.HIGH)
        self._ioe.output(self._pin2, io.HIGH)
        self._log.info('lights off.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self._ioe.output(self._pin1, io.HIGH)
        self._ioe.output(self._pin2, io.HIGH)
        self._log.info('closed.')

#EOF
