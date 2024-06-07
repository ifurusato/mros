#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-09-02
# modified: 2021-09-02
#

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Ranger(object):
    '''
    A simple range converter class, e.g.:

    _ranger = Ranger( 0, 100, -1.0, 1.0)
    _value = _ranger.convert(66)

    The returned value is converted to an integer or float depending
    on the type of the new range.
    '''
    def __init__(self, old_min, old_max, new_min, new_max):
        self._old_min = old_min
        self._old_max = old_max
        self._new_min = new_min
        self._new_max = new_max
        if type(new_min) != type(new_max):
            raise TypeError('new_min and new_max must be the same type')
        self._convert_to_int = isinstance(new_min, int) # otherwise assume float
        self._convert = lambda n : self._new_min if (self._old_max - self._old_min) == 0 \
                else (((n - self._old_min) * (self._new_max - self._new_min)) / (self._old_max - self._old_min)) + self._new_min

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def convert(self, value):
        return int(self._convert(value)) if self._convert_to_int else float(self._convert(value))

#EOF
