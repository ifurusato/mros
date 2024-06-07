#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-15
# modified: 2024-05-19
#

import sys, time, traceback
from datetime import datetime as dt
from math import isclose
from colorama import init, Fore, Style
init()

try:
    import pytest
except Exception:
    print("could not find library 'pytest': install with \"sudo pip3 install pytest\"")
    sys.exit(1)

from core.rate import Rate
from core.logger import Logger, Level
import hardware.ThunderBorg3
from hardware.ThunderBorg3 import ThunderBorg, ScanForThunderBorg, SetNewAddress
from hardware.sound import Sound, Player

_level = Level.INFO
_log = Logger('test', _level)
_tb1 = None
_tb2 = None


# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_start_time = dt.now()

MOTOR_DELAY_MS    = 100
SERVO_DELAY_MS    = 80
MAX_POWER   = 0.8

_log.info("scanning for thunderborgs...")
ScanForThunderBorg()

_tb1 = ThunderBorg(Level.INFO)
_tb1.i2cAddress = 0x16
_tb1.Init()
_tb1.SetLedShowBattery(False)

_tb2 = ThunderBorg(Level.INFO)
_tb2.i2cAddress = 0x15
_tb2.Init()
_tb2.SetLedShowBattery(False)

def set_power(power):
    _tb1.SetMotor1(power)
    _tb2.SetMotor1(power)
    _tb1.SetMotor2(power)
    _tb2.SetMotor2(power)

_log.info("start...")

try:

    _log.info('starting motor test...')

    Player().play(Sound.BLIP)

    # go there...
    for power in map(lambda x: x/100.0, range(0, 50, 1)):
        set_power(power)
        time.sleep(MOTOR_DELAY_MS/ 1000)
    for power in map(lambda x: x/100.0, range(50, 0, -1)):
        set_power(power)
        time.sleep(MOTOR_DELAY_MS/ 1000)

    # and back...
    for power in map(lambda x: x/100.0, range(0, 50, 1)):
        set_power(-1 * power)
        time.sleep(MOTOR_DELAY_MS/ 1000)
    for power in map(lambda x: x/100.0, range(50, 0, -1)):
        set_power(-1 * power)
        time.sleep(MOTOR_DELAY_MS/ 1000)

    _tb1.SetMotors(0.0)
    _tb2.SetMotors(0.0)

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting...')
except Exception as e:
    _log.error('{} encountered, exiting: {}'.format(type(e), e))
finally:
    if _tb1:
        _tb1.MotorsOff()
        _tb1.SetLeds(0.0, 0.0, 0.0) # black
    if _tb2:
        _tb2.MotorsOff()
        _tb2.SetLeds(0.0, 0.0, 0.0) # black

Player().play(Sound.BELL)

_elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
_log.info(Fore.YELLOW + 'complete: elapsed: {:d}ms'.format(_elapsed_ms))


#EOF
