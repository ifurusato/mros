#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-14
# modified: 2024-05-14
#

import ioexpander as io
from ioexpander.encoder import Encoder

from core.logger import Level, Logger

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class RotaryEncoder(object):

    I2C_ADDR = 0x0F  # 0x18 for IO Expander, 0x0F for the encoder breakout

    CHANNEL = 1
    PIN_RED   = 1
    PIN_GREEN = 7
    PIN_BLUE  = 2

    POT_ENC_A = 12
    POT_ENC_B = 3
    POT_ENC_C = 11
    PINS = [ POT_ENC_A, POT_ENC_B ]

    BRIGHTNESS = 0.9                # Effectively the maximum fraction of the period that the LED will be on
    PERIOD = int(255 / BRIGHTNESS)  # Add a period large enough to get 0-255 steps at the desired brightness

    def __init__(self, level=Level.INFO):
        self._log = Logger("rot-enc", level)

        self._ioe = io.IOE(i2c_addr=RotaryEncoder.I2C_ADDR, interrupt_pin=4)
        self._ioe.enable_interrupt_out(pin_swap=True)
        self._callback = None

        self._encoder = Encoder(self._ioe, RotaryEncoder.CHANNEL, RotaryEncoder.PINS, common_pin=RotaryEncoder.POT_ENC_C)
        # configure for RGB LED display
        self._ioe.set_pwm_period(RotaryEncoder.PERIOD)
        self._ioe.set_pwm_control(divider=2)  # PWM as fast as we can to avoid LED flicker
        self._ioe.set_mode(RotaryEncoder.PIN_RED, io.PWM, invert=True)
        self._ioe.set_mode(RotaryEncoder.PIN_GREEN, io.PWM, invert=True)
        self._ioe.set_mode(RotaryEncoder.PIN_BLUE, io.PWM, invert=True)
        self._log.info("ready: rotary encoder running LED with {} brightness steps.".format(
                int(RotaryEncoder.PERIOD * RotaryEncoder.BRIGHTNESS)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_callback(self, callback):
        '''
        Sets the callback to be executed upon an interrupt.
        '''
        self._ioe.on_interrupt(callback)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def delta(self):
        return self._encoder.delta()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def value(self):
        return self._encoder.step()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_interrupt(self):
        return self._ioe.get_interrupt()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_rgb(self, rgb):
        self.set_color(int(rgb[0]), int(rgb[1]), int(rgb[2]))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_color(self, r, g, b):
        self._ioe.output(RotaryEncoder.PIN_RED, r)
        self._ioe.output(RotaryEncoder.PIN_GREEN, g)
        self._ioe.output(RotaryEncoder.PIN_BLUE, b)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self.set_color(0,0,0)
        self._ioe.clear_interrupt()

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Selector(object):
    '''
    Provided an incrementing or decrementing value, cycles through the values
    between 0 and the stated limit. This is similar to itertools.cycle() but
    also permits decrementing.
    '''

    def __init__(self, limit, level=Level.INFO):
        self._limit = limit
        self._value = 0
        self._last_value = 0

    def set_limit(self, limit):
        self._limit = limit - 1

    def get_value(self, value):
        if value > self._last_value: # count up
            self._value += 1
        elif value < self._last_value: # count down
            self._value -= 1
        if self._value < 0:
            self._value = self._limit
        elif self._value > self._limit:
            self._value = 0
        self._last_value = value
        return self._value

#EOF
