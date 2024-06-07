#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-04-27
# modified: 2024-06-08
#
# An enumeration of various slew rates.
#

from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SlewRate(Enum): # tested to 50.0 velocity:
    #                  num   ratio     pid    limit
#   SLOWEST          = ( 0,  0.009,   0.16, 00.0001 ) # 5.1 sec
    SLOWEST          = ( 0,   0.02,   0.22, 00.0002 ) # 3.1 sec
    SLOWER           = ( 1,   0.05,   0.38, 00.0005 ) # 1.7 sec
    SLOW             = ( 2,   0.08,   0.48, 00.0010 ) # 1.3 sec
    NORMAL           = ( 3,   0.10,   0.58, 00.0050 ) # 1.0 sec
    FAST             = ( 4,   0.25,   0.68, 00.0100 ) # 0.6 sec
    FASTER           = ( 5,   0.50,   0.90, 00.0200 ) # 0.5 sec
    FASTEST          = ( 6,   0.95,   0.90, 00.1000 ) # 0.9 sec

    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ignore the first param since it's already set by __new__
    def __init__(self, num, ratio, pid, limit):
        self._ratio = ratio
        self._pid   = pid
        self._limit = limit

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def label(self):
        return self.name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_string(value):
        for r in SlewRate:
            if value.upper() == r.name:
                return r
        raise ValueError('unrecognised value for slew rate: {}'.format(value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def ratio(self):
        return self._ratio

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def limit(self):
        return self._limit

#EOF
