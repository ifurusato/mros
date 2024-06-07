#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-10
# modified: 2024-05-10
#

import sys, time, traceback
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

import hardware.ThunderBorg3
from hardware.ThunderBorg3 import ThunderBorg, ScanForThunderBorg, SetNewAddress

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_tb1 = None
_tb2 = None

try:
    ENABLE_TB1     = True
    ENABLE_TB2     = True
    LED_TEST_TB1   = False
    LED_TEST_TB2   = False
    MOTOR_TEST_TB1 = True
    MOTOR_TEST_TB2 = True
    DELAY_SEC      = 3
    MOTOR_POWER    = 0.5

    _log = Logger('thun', Level.INFO)
    _log.info("scanning for thunderborgs...")

    ScanForThunderBorg()

    _log.info("start...")

#   SetNewAddress(0x16)

    if ENABLE_TB1:
        _tb1 = ThunderBorg(Level.INFO)
        _tb1.i2cAddress = 0x16
        _tb1.Init()

    if ENABLE_TB2:
        _tb2 = ThunderBorg(Level.INFO)
        _tb2.i2cAddress = 0x15
        _tb2.Init()

    # thunderborg 1 ...............................
    if _tb1:
        _tb1.SetLedShowBattery(False)
    
        if LED_TEST_TB1:
            _log.info('ThunderBorg 1 LED Test…')
            for _ in range(7):
                _tb1.SetLed1(0.1, 0.0, 0.1) # ThunderBorg LED dull purple
                time.sleep(0.2)
                _tb1.SetLed1(0.7, 0.0, 0.7) # ThunderBorg LED dull purple
                time.sleep(0.2)
            _tb1.SetLeds(1.0, 0.0, 0.0) # -> starboard ThunderBorg LED red
            time.sleep(3)
            _tb1.SetLeds(0.0, 0.0, 0.0) # black
    
        if MOTOR_TEST_TB1:
            _log.info(Fore.RED + 'ThunderBorg 1 Motor 1 Test…' + Style.RESET_ALL)
            _tb1.SetMotor1(MOTOR_POWER) # fore port
            time.sleep(DELAY_SEC)
            _tb1.SetMotor1(0.0)
    
            _log.info(Fore.GREEN + 'ThunderBorg 1 Motor 2 Test…' + Style.RESET_ALL)
            _tb1.SetMotor2(MOTOR_POWER) # fore starboard
            time.sleep(DELAY_SEC)
            _tb1.SetMotor2(0.0)

    # thunderborg 2 ...............................
    if _tb2:
        _tb2.SetLedShowBattery(False)

        if LED_TEST_TB2:
            _log.info('ThunderBorg 2 LED Test…')
            for _ in range(7):
                _tb2.SetLed1(0.1, 0.0, 0.1) # ThunderBorg LED dull purple
                time.sleep(0.2)
                _tb2.SetLed1(0.7, 0.0, 0.7) # ThunderBorg LED dull purple
                time.sleep(0.2)
            _tb2.SetLeds(1.0, 0.0, 0.0) # -> starboard ThunderBorg LED red
            time.sleep(3)
            _tb2.SetLeds(0.0, 0.0, 0.0) # black
    
        if MOTOR_TEST_TB2:
            _log.info(Fore.RED + 'ThunderBorg 2 Motor 1 (PORT) Test…' + Style.RESET_ALL)
            _tb2.SetMotor1(MOTOR_POWER)
            time.sleep(DELAY_SEC)
            _tb2.SetMotor1(0.0)
    
            _log.info(Fore.GREEN + 'ThunderBorg 2 Motor 2 (STBD) Test…' + Style.RESET_ALL)
            _tb2.SetMotor2(MOTOR_POWER)
            time.sleep(DELAY_SEC)
            _tb2.SetMotor2(0.0)

    _log.info("done.")

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting...')
except Exception as e:
    _log.error('{} thrown in thunderborg test: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    if _tb1:
        _tb1.MotorsOff()
#       _tb1.SetLedShowBattery(True)
        _tb1.SetLeds(0.0, 0.0, 0.0) # black
    if _tb2:
        _tb2.MotorsOff()
#       _tb2.SetLedShowBattery(True)
        _tb2.SetLeds(0.0, 0.0, 0.0) # black
    pass

#EOF
