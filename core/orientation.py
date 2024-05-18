#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2021 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2024-05-18
#
# An enum for expressing different orientations.
#

from enum import Enum
from hardware.color import Color

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Orientation(Enum):
    NONE  = (  0, "none",           "none")
    PORT  = (  1, "port",           "port")
    CNTR  = (  2, "center",         "cntr")
    STBD  = (  3, "starboard",      "stbd")
    FORE  = (  4, "fore",           "fore")
    MID   = (  5, "mid",            "mid")
    AFT   = (  6, "aft",            "aft")
    PSID  = (  7, "port-side",      "psid")
    SSID  = (  8, "stbd-side",      "ssid")
    PFOR  = (  9, "port-fore",      "pfor")
    SFOR  = ( 10, "starboard-fore", "sfor")
    PMID  = ( 11, "port-mid",       "pmid")
    SMID  = ( 12, "starboard-mid",  "smid")
    PAFT  = ( 13, "port-aft",       "paft")
    SAFT  = ( 14, "starboard-aft",  "saft")
    MAST  = ( 15, "mast",           "mast")

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, label):
        self._name = name
        self._label = label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        '''
        Return the name. This makes sure the name is read-only.
        '''
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def label(self):
        '''
        Return the label. This makes sure the label is read-only.
        '''
        return self._label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_label(label):
        '''
        Returns the Orientation matching the label or None.
        '''
        for o in Orientation:
            if label == o.label:
                return o
        return None

#EOF
