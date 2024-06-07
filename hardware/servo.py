#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-13
# modified: 2024-06-03
#
# Requires installation of the Adafruit Servokit library:
#
#    sudo pip3 install adafruit-circuitpython-servokit
#

import sys
from enum import Enum
import time

try:
    from adafruit_servokit import ServoKit
except Exception:
    print("could not find library 'adafruit-circuitpython-servokit': install with \"sudo pip3 install adafruit-circuitpython-servokit\"")
    sys.exit(1)

from datetime import datetime as dt
from math import isclose
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

_kit = ServoKit(channels=16)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Servo(object):
    '''
    A wrapper around a physical servo in a specific position (provided
    as an Orientation), with center trim and set limits.
    '''
    def __init__(self, config, orientation, enabled, level):
        self._name = orientation.label
        self._log = Logger('servo-{}'.format(self._name), level)
        _label = orientation.label 
        self._scale_factor = config.get('scale_factor') # 0.64
        self._max_range = config.get('max_range') # 135
        self._trim  = config.get('servo-{}-trim'.format(_label))
        _channel    = config.get('servo-{}-channel'.format(_label))
        self._servo = _kit.servo[_channel]
        self._min   = config.get('servo-{}-min'.format(_label))
        self._max   = config.get('servo-{}-max'.format(_label))
#       self._clamp = lambda n: self._min if n < self._min else self._max if n > self._max else n
        self._clamp = lambda n: max(min(self._max, n), self._min)
#       self._clamp = lambda n: max(min(self._max_range, n), 0)
        self._servo.actuation_range = self._max_range
        self._angle = 0
        self._enabled = enabled
        if 'p' in self._name:
            self._color = Fore.RED
        else:
            self._color = Fore.GREEN
        self._code = self._get_code()
        self._log.info(Fore.CYAN + "{} servo set up on channel {}.".format(self._name, _channel) + Style.RESET_ALL)

    @property
    def name(self):
        return self._name

    @property
    def trim(self):
        return self._trim

#   @property
#   def servo(self):
#       return self._servo

    @property
    def angle(self):
        '''
        Returns the last angle set for this servo; zero if never set.
        '''
        return self._angle

    @property
    def enabled(self):
        return self._enabled

    def set_enabled(self, enabled):
        self._enabled = enabled

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

#   def clamp(self, value):
#       '''
#       Return the clamp lambda function. This is the same for any servo.
#       '''
#       return self._clamp(value)

    def center(self):
        if self._enabled:
            self._log.info(self._color + Style.BRIGHT + 'center {} servo position.'.format(self._name) + Style.RESET_ALL)
            self.set_angle(0)

    def set_angle(self, angle):
        '''
        Set the angle of the servo to the specified value, generally a sweep of ±90°.
        This is scaled and clamped for safety.
        '''
        if self._enabled:
            self._angle = int(self._clamp(angle) * self._scale_factor)
            _target_angle = int(self._angle + self._trim)

#           self._angle = int(angle * self._scale_factor)
#           _target_angle = int(self._clamp(self._angle) + self._trim)

            self._servo.angle = _target_angle
            self._log.info(self._code + Fore.CYAN + ' for {} servo: processed to {:.2f}° (called with'.format(self._name, self._angle) 
                    + Style.BRIGHT + ' {:.2f}°'.format(angle) 
                    + Style.NORMAL + ') set to value: {:.2f}; trim: {:.2f}; min/max: {:.2f}/{:.2f}'.format(self._servo.angle, self._trim, self._min, self._max))

    def _get_code(self):
        if self._name == 'pfwd':
            return Fore.RED + '██▒' + Fore.YELLOW + '██▒' # ▒ 
        elif self._name == 'sfwd':
            return Fore.GREEN + '██▒' + Fore.YELLOW + '██▒'
        elif self._name == 'pmid':
            return Fore.RED + '██▒' + Fore.MAGENTA + '██▒'
        elif self._name == 'smid':
            return Fore.GREEN + '██▒' + Fore.MAGENTA + '██▒'
        elif self._name == 'paft':
            return Fore.RED + '██▒' + Fore.CYAN + '██▒'
        elif self._name == 'saft':
            return Fore.GREEN + '██▒' + Fore.CYAN + '██▒'

#EOF
