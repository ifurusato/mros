#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-01
# modified: 2021-10-16
#
# An enumeration of the names of numbers through twenty.
#

from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Numbers(Enum):
    ZERO      = ( 0,  'zero' )
    ONE       = ( 1,  'one' )
    TWO       = ( 2,  'two' )
    THREE     = ( 3,  'three' )
    FOUR      = ( 4,  'four' )
    FIVE      = ( 5,  'five' )
    SIX       = ( 6,  'six' )
    SEVEN     = ( 7,  'seven' )
    EIGHT     = ( 8,  'eight' )
    NINE      = ( 9,  'nine' )
    TEN       = ( 10, 'ten' )
    ELEVEN    = ( 11, 'eleven' )
    TWELVE    = ( 12, 'twelve' )
    THIRTEEN  = ( 13, 'thirteen' )
    FOURTEEN  = ( 14, 'fourteen' )
    FIFTEEN   = ( 15, 'fifteen' )
    SIXTEEN   = ( 16, 'sixteen' )
    SEVENTEEN = ( 17, 'seventeen' )
    EIGHTEEN  = ( 18, 'eighteen' )
    NINETEEN  = ( 19, 'nineteen' )
    TWENTY    = ( 20, 'twenty' )

    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name):
        self._name = name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self.name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_number(num):
        '''
        Returns 'zero' through 'twenty', then simply a string
        version of the number.
        '''
        for n in Numbers:
            if n.value == num:
                return n._name
        return str(num)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __eq__(self, obj):
        return isinstance(obj, Number) and obj.value == self.value

#EOF
