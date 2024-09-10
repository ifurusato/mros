#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2024-09-07
# modified: 2024-09-07
#
# Uses two pins to determine if one of three LED channels is to be turned on.
#

from machine import Pin
from picofx import Updateable

class TrioFX(Updateable):
    def __init__(self, channel=0, pin0=16, pin1=17, interval=0.1, brightness=1.0, additive=True):
        self.interval = interval
        self._pin0 = Pin(pin0, Pin.IN, Pin.PULL_UP)
        self._pin1 = Pin(pin1, Pin.IN, Pin.PULL_UP)
        self._channel = channel
        self._additive = additive
        self._brightness = brightness
        self.__value = 0.0
        self.__time = 0

    def set(self, value):
        self.__value = value

    def tick(self, delta_ms):
        self.__time += delta_ms
        # check if the interval has elapsed
        if self.__time >= (self.interval * 1000):
            self.__time -= (self.interval * 1000)
            _v0 = int(self._pin0.value())
            _v1 = int(self._pin1.value())
            # we reverse the logic so low is True
            _v0 = 0 if _v0 == 1 else 1
            _v1 = 0 if _v1 == 1 else 1
            _decimal = int('{}{}'.format(_v1, _v0), 2)
            if _decimal == 0:
                self.__value = 0.0
            elif _decimal == self._channel:
                self.__value = self._brightness
            elif not self._additive:
                self.__value = 0.0
            else:
                pass

    def __call__(self):
        return self.__value

#EOF
