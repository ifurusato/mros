#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-28
# modified: 2024-05-28
#

import sys
from enum import Enum

try:
    import RPi.GPIO as GPIO
except Exception:
    print('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
    sys.exit(1)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GpioMode(Enum):
    __order__ = " OUT IN SERIAL SPI   I2C  HARD_PWM UNKNOWN "
    '''
    Enumerates the set of GPIO modes.
    '''
    OUT       = (  0, "GPIO.OUT",      GPIO.OUT )
    IN        = (  1, "GPIO.IN",       GPIO.IN )
    SERIAL    = ( 40, "GPIO.SERIAL",   GPIO.SERIAL )
    SPI       = ( 41, "GPIO.SPI",      GPIO.SPI )
    I2C       = ( 42, "GPIO.I2C",      GPIO.I2C )
    HARD_PWM  = ( 43, "GPIO.HARD_PWM", GPIO.HARD_PWM )
    UNKNOWN   = ( -1, "GPIO.UNKNOWN",  GPIO.UNKNOWN )

    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ignore the first param since it's already set by __new__
    def __init__(self, num, label, mode):
        self._num   = num
        self._label = label
        self._mode  = mode

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def num(self):
        '''
        Returns the original enum numerical value for the mode.
        '''
        return self._num

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def label(self):
        '''
        Returns a display label for the mode.
        '''
        return self._label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mode(self):
        '''
        Returns the original GPIO Enum value.
        '''
        return self._mode

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_function(pin):
        '''
        Returns the GPIO mode for the given pin, using BCM numbering.
        '''
        GPIO.setmode(GPIO.BCM)
        _mode = GPIO.gpio_function(pin)
        if _mode is GPIO.OUT:
            return GpioMode.OUT
        elif _mode is GPIO.IN:
            return GpioMode.IN
        elif _mode is GPIO.SERIAL:
            return GpioMode.SERIAL
        elif _mode is GPIO.SPI:
            return GpioMode.SPI
        elif _mode is GPIO.I2C:
            return GpioMode.I2C
        elif _mode is GPIO.HARD_PWM:
            return GpioMode.HARD_PWM
        elif _mode is GPIO.UNKNOWN:
            return GpioMode.UNKNOWN
        else:
            raise Exception('unrecognised GPIO mode: {}'.format(_mode))

#EOF
