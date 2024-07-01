#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-05
# modified: 2024-06-15
#
# This is where events can be mapped to other events:
#
#     HOME_BUTTON = ( 11, 306, 'home', 'Home Button', Event.SHUTDOWN)
#

from enum import Enum

from core.event import Event

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class GamepadMapping(Enum):
    '''
    Maps Gamepad buttons to Events.

    An enumeration of the controls available on the 8BitDo N30 Pro Gamepad,
    or any similar/compatible model. The numeric values for 'code' may need
    to be modified for different devices, but the basic functionality of this
    Enum should hold.

    This also includes an Event variable, which provides the mapping between
    a specific gamepad control and its corresponding action.

    The @property annotations make sure the respective variable is read-only.

    control            num  code  id          control descripton      event               notes/alt mapping
    '''
    A_BUTTON        = ( 1,  304,  'cross',    'A (Cross) Button',     Event.A_BUTTON)
    B_BUTTON        = ( 2,  305,  'circle',   'B (Circle) Button',    Event.B_BUTTON)     # noop
    X_BUTTON        = ( 3,  307,  'triangle', 'X (Triangle) Button',  Event.STOP)         # escalating stop
    Y_BUTTON        = ( 4,  308,  'square',   'Y ((Square) Button',   Event.SHUTDOWN)

    L1_BUTTON       = ( 5,  310,  'l1',       'L1 Button',            Event.L1_BUTTON)
    L2_BUTTON       = ( 6,  312,  'l2',       'L2 Button',            Event.L2_BUTTON)
    R1_BUTTON       = ( 8,  311,  'r1',       'R1 Button',            Event.R1_BUTTON)
    R2_BUTTON       = ( 7,  313,  'r2',       'R2 Button',            Event.R2_BUTTON)

    START_BUTTON    = ( 9,  315,  'start',    'Start Button',         Event.START_BUTTON)
    SELECT_BUTTON   = ( 10, 314,  'select',   'Select Button',        Event.SELECT_BUTTON)
    HOME_BUTTON     = ( 11, 306,  'home',     'Home Button',          Event.HOME_BUTTON)

    DPAD_HORIZONTAL = ( 12, 16,   'dph',      'D-PAD Horizontal',     Event.DPAD_HORIZONTAL)
    DPAD_LEFT       = ( 12, 16,   'dpl',      'D-PAD Left',           Event.DPAD_LEFT)
    DPAD_RIGHT      = ( 13, 16,   'dpr',      'D-PAD Right',          Event.DPAD_RIGHT)
    DPAD_VERTICAL   = ( 14, 17,   'dpv',      'D-PAD Vertical',       Event.DPAD_VERTICAL)
    DPAD_UP         = ( 14, 17,   'dpu',      'D-PAD Up',             Event.DPAD_UP)
    DPAD_DOWN       = ( 15, 17,   'dpd',      'D-PAD Down',           Event.DPAD_DOWN)

    L3_VERTICAL     = ( 16, 1,    'l3v',      'L3 Vertical',          Event.L3_VERTICAL)
    L3_HORIZONTAL   = ( 17, 0,    'l3h',      'L3 Horizontal',        Event.L3_HORIZONTAL)
    R3_VERTICAL     = ( 18, 5,    'r3v',      'R3 Vertical',          Event.R3_VERTICAL)
    R3_HORIZONTAL   = ( 19, 2,    'r3h',      'R3 Horizontal',        Event.R3_HORIZONTAL)

    # ignore the first param since it's already set by __new__
    def __init__(self, num, code, name, label, event):
        self._code = code
        self._name = name
        self._label = label
        self._event = event

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def code(self):
        return self._code

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def label(self):
        return self._label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def event(self):
        return self._event

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_by_code(self, event):
        '''
        Return the Gamepad mapping for the event's code, with a special
        case for DPAD events.
        '''
        _code = event.code
        if _code == GamepadMapping.DPAD_HORIZONTAL.code:
            if event.value == 1:
                return GamepadMapping.DPAD_RIGHT
            elif event.value == -1:
                return GamepadMapping.DPAD_LEFT
        elif _code == GamepadMapping.DPAD_VERTICAL.code:
            if event.value == -1:
                return GamepadMapping.DPAD_UP
            elif event.value == 1:
                return GamepadMapping.DPAD_DOWN
        else:
            for ctrl in GamepadMapping:
                if ctrl.code == _code:
                    return ctrl
        return None

# EOF
