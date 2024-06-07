#}!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-29
# modified: 2024-05-19
#
# Note that stopped, clockwise and counter-clockwise are descriptive, not prescriptive.
#

from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Direction(Enum):
    STOPPED           = (  0, 'stopped',           'stop')
    AHEAD             = (  1, 'ahead',             'ahed')
    ASTERN            = (  2, 'astern',            'astn')
    CLOCKWISE         = (  3, 'clockwise',         'clws')
    COUNTER_CLOCKWISE = (  4, 'counter-clockwise', 'ccwz')
    UNKNOWN           = ( 99, 'unknown',           'unkn') # n/a or indeterminate

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, label):
        self._name  = name
        self._label = label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def label(self):
        return self._label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_direction_for(port_velocity, stbd_velocity):
        '''
        NOTE: Not yet implemented for enums > 4.
        '''
        if port_velocity and stbd_velocity:
            if port_velocity == 0.0 and stbd_velocity == 0.0:
                return Direction.STOPPED
            elif port_velocity > 0.0 and stbd_velocity > 0.0:
                return Direction.AHEAD
            elif port_velocity < 0.0 and stbd_velocity < 0.0:
                return Direction.ASTERN
            elif port_velocity > 0.0 and stbd_velocity <= 0.0:
                return Direction.CLOCKWISE
            elif port_velocity <= 0.0 and stbd_velocity > 0.0:
                return Direction.COUNTER_CLOCKWISE
            else:
                raise TypeError('unable to discern direction for port: {}; stbd: {}'.format(port_velocity, stbd_velocity))
        else:
            return Direction.UNKNOWN

#EOF
