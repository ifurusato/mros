#}!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-06-20
# modified: 2024-06-20

# The terms used here are derived from the "engine order telegraph" or
# Chadburn used to describe speed on ships.
#

from math import isclose
from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Chadburn(Enum):
    __order__ = " EMERGENCY_ASTERN FULL_ASTERN THREE_QUARTER_ASTERN TWO_THIRDS_ASTERN HALF_ASTERN ONE_THIRD_ASTERN SLOW_ASTERN DEAD_SLOW_ASTERN STOP DEAD_SLOW_AHEAD SLOW_AHEAD ONE_THIRD_AHEAD HALF_AHEAD TWO_THIRDS_AHEAD THREE_QUARTER_AHEAD FULL_AHEAD FLANK_AHEAD "
    '''
    Provides an enumeration of both ahead (forward) and astern (reverse)
    Chadburn-style speeds, as corresponding to an abstract speed value.
    This is similar to the Speed enum and is used solely for stepping
    through Chadburn speeds using the Gamepad.

    The values for astern and ahead proportional power are initially set
    as equal, but may need to be tweaked.
    '''
    #                       num   name                     speed
    EMERGENCY_ASTERN     = ( -8, 'emergency astern',       -1.00 )
    FULL_ASTERN          = ( -7, 'full astern',            -0.85 )
    THREE_QUARTER_ASTERN = ( -6, 'three quarters astern',  -0.70 )
    TWO_THIRDS_ASTERN    = ( -5, 'two thirds astern',      -0.62 )
    HALF_ASTERN          = ( -4, 'half astern',            -0.46 )
    ONE_THIRD_ASTERN     = ( -3, 'one third astern',       -0.39 )
    SLOW_ASTERN          = ( -2, 'slow astern',            -0.21 )
    DEAD_SLOW_ASTERN     = ( -1, 'dead slow astern',       -0.07 )
    STOP                 = (  0, 'stop',                    0.00 )
    DEAD_SLOW_AHEAD      = (  1, 'dead slow ahead',         0.07 )
    SLOW_AHEAD           = (  2, 'slow ahead',              0.21 )
    ONE_THIRD_AHEAD      = (  3, 'one third ahead',         0.39 )
    HALF_AHEAD           = (  4, 'half ahead',              0.46 )
    TWO_THIRDS_AHEAD     = (  5, 'two thirds ahead',        0.62 )
    THREE_QUARTER_AHEAD  = (  6, 'three quarters ahead',    0.70 )
    FULL_AHEAD           = (  7, 'full ahead',              0.85 )
    FLANK_AHEAD          = (  8, 'flank ahead',             1.00 )

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, speed):
        self._num   = num
        self._name  = name
        self._speed = speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def num(self):
        '''
        Returns the original enum numerical value for the mode.
        '''
        return self._num

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def value(self):
        raise Exception('can\'t call value directly.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def speed(self):
        '''
        Return the numeric value for this Chadburn speed.
        '''
        return self._speed

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_index(value):
        for c in Chadburn:
            if value == c.num:
                return c
        raise NotImplementedError

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_closest_value(value):
        '''
        Return the closest enumerated value to the argument, opting for the
        absolute value lower than the argument.
        '''
        if isclose(value, 0.0, abs_tol=1e-2):
            return Chadburn.STOP
        elif value < -1.0:
            return Chadburn.FULL_ASTERN
        elif value > 1.0:
            return Chadburn.FULL_AHEAD
        elif value < 0:
            for _chadburn in Chadburn:
                if value <= _chadburn.speed:
                    return _chadburn
        else:
            for _chadburn in reversed(Chadburn):
                if value >= _chadburn.speed:
                    return _chadburn

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        return 'Chadburn.{}; speed={:5.2f}'.format(self.name, self._speed )

#EOF
