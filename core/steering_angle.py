#}!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-06-22
# modified: 2024-06-22
#

from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SteeringAngle(Enum):
    __order__ = " COUNTER_CLOCKWISE_45 COUNTER_CLOCKWISE_35 COUNTER_CLOCKWISE_25 COUNTER_CLOCKWISE_15 COUNTER_CLOCKWISE_5 STRAIGHT_AHEAD CLOCKWISE_5 CLOCKWISE_15 CLOCKWISE_25 CLOCKWISE_35 CLOCKWISE_45 "
    '''
    Provides a stepped enumeration of steering angles between the
    limits of counter-clockwise at -45° and clockwise at 45°.
    '''
    COUNTER_CLOCKWISE_45  = ( -5, 'counter-clockwise-45', -45)
    COUNTER_CLOCKWISE_35  = ( -4, 'counter-clockwise-35', -35)
    COUNTER_CLOCKWISE_25  = ( -3, 'counter-clockwise-25', -25)
    COUNTER_CLOCKWISE_15  = ( -2, 'counter-clockwise-15', -15)
    COUNTER_CLOCKWISE_5   = ( -1, 'counter-clockwise-5',   -5)
    STRAIGHT_AHEAD        = (  0, 'straight-ahead',         0)
    CLOCKWISE_5           = (  1, 'clockwise-5',            5)
    CLOCKWISE_15          = (  2, 'clockwise-15',          15)
    CLOCKWISE_25          = (  3, 'clockwise-25',          25)
    CLOCKWISE_35          = (  4, 'clockwise-35',          35)
    CLOCKWISE_45          = (  5, 'clockwise-45',          45)

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, value):
        self._num   = num
        self._name  = name
        self._value = value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def num(self):
        '''
        Returns the original enum numerical value for the steering angle.
        '''
        return self._num

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def value(self):
        return self._value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_index(value):
        for c in SteeringAngle:
            if value == c.num:
                return c
        raise NotImplementedError

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        return 'SteeringAngle.{}; value={:4.2f}'.format(self.name, self._value )

#EOF
