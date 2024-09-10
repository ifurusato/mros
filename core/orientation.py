#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2024-06-07
#
# An enum for expressing different orientations.
#

from enum import Enum

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Orientation(Enum):
    NONE  = (  0, "none",          'NONE',  "none")
    PORT  = (  1, "port",          'PORT',  "port")
    CNTR  = (  2, "center",        'NONE',  "cntr")
    STBD  = (  3, "starboard",     'STBD',  "stbd")
    FWD   = (  4, "fwd",           'NONE',  "fwd")
    MID   = (  5, "mid",           'NONE',  "mid")
    AFT   = (  6, "aft",           'NONE',  "aft")
    PSID  = (  7, "port-side",     'PORT',  "psid")
    SSID  = (  8, "stbd-side",     'STBD',  "ssid")
    PFWD  = (  9, "port-fwd",      'PORT',  "pfwd")
    SFWD  = ( 10, "starboard-fwd", 'STBD',  "sfwd")
    PMID  = ( 11, "port-mid",      'PORT',  "pmid")
    SMID  = ( 12, "starboard-mid", 'STBD',  "smid")
    PAFT  = ( 13, "port-aft",      'PORT',  "paft")
    SAFT  = ( 14, "starboard-aft", 'STBD',  "saft")

    MAST  = ( 15, "mast",          'NONE',  "mast")
    CAM   = ( 16, "camera",        'NONE',  "cam")

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, side, label):
        self._name  = name
        self._side  = side
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
    @property
    def side(self):
        '''
        Return the PORT or STBD side of this orientation, NONE if it does not apply.
        '''
        if self._side == 'PORT':
            return Orientation.PORT
        if self._side == 'STBD':
            return Orientation.STBD
        else:
            return Orientation.NONE

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
