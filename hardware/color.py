#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2024-05-19
#
# An enum of colors.
#

from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Color(Enum):

    WHITE           = (  1, 255.0, 255.0, 255.0)
    LIGHT_GREY      = (  2, 192.0, 192.0, 192.0)
    GREY            = (  3, 96.0, 96.0, 96.0)
    DARK_GREY       = (  4, 64.0, 64.0, 64.0)
    VERY_DARK_GREY  = (  5, 37.0, 37.0, 37.0)
    BLACK           = (  6, 0.0, 0.0, 0.0)
    LIGHT_RED       = (  7, 255.0, 128.0, 128.0)
    RED             = (  8, 255.0, 0.0, 0.0)
    DARK_RED        = (  9, 128.0, 0.0, 0.0)
    BROWN           = ( 10, 64.0, 64.0, 0.0)
    DARK_ORANGE     = ( 10, 255.0, 50.0, 0.0)
    TANGERINE       = ( 10, 223.0, 64.0, 10.0)
    ORANGE          = ( 13, 255.0, 108.0, 0.0)
    DARK_YELLOW     = ( 14, 128.0, 128.0, 0.0)
    YELLOW          = ( 15, 255.0, 160.0, 0.0)
    LIGHT_YELLOW    = ( 16, 255.0, 180.0, 48.0)
    YELLOW_GREEN    = ( 17, 148.0, 255.0, 0.0)
    LIGHT_GREEN     = ( 18, 128.0, 255.0, 64.0)
    GREEN           = ( 19, 0.0, 255.0, 0.0)
    DARK_GREEN      = ( 20, 0.0, 96.0, 0.0)
    LIGHT_TURQUOISE = ( 21, 150.0, 199.0, 128.0)
    TURQUOISE       = ( 22, 0.0, 160.0, 96.0)
    DARK_TURQUOISE  = ( 23, 22.0, 81.0, 55.0)
    LIGHT_BLUE      = ( 24, 128.0, 128.0, 255.0)
    BLUE            = ( 25, 0.0, 40.0, 255.0)
    SKY_BLUE        = ( 26, 40.0, 128.0, 192.0)
    DARK_BLUE       = ( 27, 0.0, 0.0, 128.0)
    BLUE_VIOLET     = ( 28, 96.0, 0.0, 255.0)
    VIOLET          = ( 29, 77.0, 26.0, 177.0)
    PURPLE          = ( 30, 128.0, 0.0, 180.0)
    LIGHT_CYAN      = ( 31, 128.0, 255.0, 255.0)
    CYAN            = ( 32, 0.0, 255.0, 255.0)
    DARK_CYAN       = ( 33, 0.0, 128.0, 128.0)
    LIGHT_MAGENTA   = ( 34, 255.0, 128.0, 255.0)
    MAGENTA         = ( 35, 255.0, 0.0, 255.0)
    DARK_MAGENTA    = ( 36, 128.0, 0.0, 128.0)
    FUCHSIA         = ( 37, 255.0, 0.0, 128.0)
    CORAL           = ( 38, 202.0, 48.0, 62.0)
    PINK            = ( 39, 171.0, 69.0, 124.0)
    DARK_PINK       = ( 40, 131.0, 63.0, 81.0)
    CANDLELIGHT     = ( 41, 226.0, 122.0, 70.0)
    DIM_RED         = ( 42, 42.0, 0.0, 0.0)

    # ignore the first param since it's already set by __new__
    def __init__(self, num, red, green, blue):
        self._red = red
        self._green = green
        self._blue = blue
        self._name = self.name.replace('_', ' ').lower()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def all_colors():
        '''
        Returns a list of all colors. This is not cached so store the result,
        don't call it repeatedly.
        '''
        __colors = []
        for c in Color:
            __colors.append(c)
        return __colors

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def rgb(self):
        return [ self._red, self._green, self._blue ]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def red(self):
        return self._red

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def green(self):
        return self._green

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def blue(self):
        return self._blue

#EOF
