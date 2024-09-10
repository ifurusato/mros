#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-28
# modified: 2024-08-28
# Turns on the LED if the associated pin is brought low (0).
#

from machine import Pin
from picofx import Updateable

class PinSetFX(Updateable):
    def __init__(self, pin=None, interval=0.1):
        self.interval = interval
        self.pin = Pin(pin, Pin.IN, Pin.PULL_UP)
        self.__value = 0.0
        self.__time = 0

    def __call__(self):
        return self.__value

    def set(self, value):
        '''
        Manually set the brightness between 0.0 and 1.0.
        '''
        self.__value = value

    def tick(self, delta_ms):
        self.__time += delta_ms

        # Check if the interval has elapsed
        if self.__time >= (self.interval * 1000):
            self.__time -= (self.interval * 1000)
            if self.pin.value() == 0:
                self.__value = 1.0
            else:
                self.__value = 0.0

