#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2024-07-15
#
# An enum for expressing the Cardinal directions.
#

from enum import Enum
from math import pi as π
from hardware.color import Color

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Cardinal(Enum):
    NORTH     = ( 0, 'north',        0, 0.0 )
    NORTHEAST = ( 1, 'north-east',  45, π * 0.25 )
    EAST      = ( 2, 'east',        90, π * 0.50 )
    SOUTHEAST = ( 3, 'south-east', 135, π * 0.75 )
    SOUTH     = ( 4, 'south',      180, π )
    SOUTHWEST = ( 5, 'south-west', 225, π * 1.25 )
    WEST      = ( 6, 'west',       270, π * 1.50 )
    NORTHWEST = ( 7, 'north-west', 315, π * 1.75 )
    UNKNOWN   = ( 8, 'unknown',     -1, -1.0 )

    # ignore the first param since it's already set by __new__
    def __init__(self, num, label, degrees, radians):
        self._label = label
        self._degrees = degrees
        self._radians = radians

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def label(self):
        return self._label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def degrees(self):
        '''
        Return the compass angle in degrees.
        '''
        return self._degrees

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def radians(self):
        '''
        Return the compass angle in radians.
        '''
        return self._radians

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_heading_from_degrees(degrees):
        '''
        Provided a heading in degrees return an enumerated cardinal direction.
        '''
        _value = round((degrees / 45.0) + 0.5)
        _array = [ Cardinal.NORTH, Cardinal.NORTHEAST, Cardinal.EAST, Cardinal.SOUTHEAST, Cardinal.SOUTH, Cardinal.SOUTHWEST, Cardinal.WEST, Cardinal.NORTHWEST ]
        return _array[(_value % 8)];

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_heading_from_degrees_old(degrees):
        '''
        Provided a heading in degrees return an enumerated cardinal direction.
        '''
        if 0 <= degrees <= 67.5:
            return Cardinal.NORTHEAST
        elif 67.5  <= degrees <= 112.5:
            return Cardinal.EAST
        elif degrees > 337.25 or degrees < 22.5:
            return Cardinal.NORTH
        elif 292.5 <= degrees <= 337.25:
            return Cardinal.NORTHWEST
        elif 247.5 <= degrees <= 292.5:
            return Cardinal.WEST
        elif 202.5 <= degrees <= 247.5:
            return Cardinal.SOUTHWEST
        elif 157.5 <= degrees <= 202.5:
            return Cardinal.SOUTH
        elif 112.5 <= degrees <= 157.5:
            return Cardinal.SOUTHEAST
        else:
            return Cardinal.UNKNOWN

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_color_for_direction(value):
        if value is Cardinal.NORTH:
            return Color.BLUE
        elif value is Cardinal.NORTHEAST:
            return Color.MAGENTA
        elif value is Cardinal.EAST:
            return Color.FUCHSIA
        elif value is Cardinal.SOUTHEAST:
            return Color.RED
        elif value is Cardinal.SOUTH:
            return Color.YELLOW
        elif value is Cardinal.SOUTHWEST:
            return Color.GREEN
        elif value is Cardinal.WEST:
            return Color.LIGHT_BLUE
        elif value is Cardinal.NORTHWEST:
            return Color.CYAN
        else:
            return Color.BLACK

#EOF
