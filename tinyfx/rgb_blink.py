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

from picofx import Cycling

import itertools
from colors import*

class RgbBlinkFX(Cycling):
    '''
    Blinks the RGB LED according to the speed, phase and duty cycle.

    The color argument is a single tuple of the form (i,R,G,B), or a
    list of such tuples. A None color argument defaults to red.

    :param:   speed   the speed of the blink, where 1.0 is 1 second, 0.5 is 2 seconds, etc.
    :param:   phase   the phase of the blink
    :param:   duty    the duty cycle of the blink
    :param:   color   the color or colors of the blink
    '''
    def __init__(self, speed=1, phase=0.0, duty=0.5, color=None):
        super().__init__(speed)
        self.phase = phase
        self.duty  = duty
        self._colors = []
        self._counter = itertools.count()
        if color is None:
            self._colors.append(COLOR['RED'])
        elif isinstance(color, tuple):
            self._colors.append(color)
        elif type(color) is list:
            self._colors.extend(color)
        else:
            raise Exception('unrecognised argument type.')
        self._cycle = itertools.cycle(self._colors)
        self._color = self._colors[0]

    def __call__(self):
        percent = (self.__offset + self.phase) % 1.0
        if percent < self.duty:
            if len(self._colors) == 1:
                self._color = self._colors[0]
            elif next(self._counter) % 5 == 0:
                self._color = next(self._cycle)
            _red   = self._color[0]
            _green = self._color[1]
            _blue  = self._color[2]
            return _red, _green, _blue
        else:
            return 0, 0, 0

