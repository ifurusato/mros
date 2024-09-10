#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-10
# modified: 2024-05-31
#
# Turns off the screen and both of the ThunderBorg LEDs, checking for the
# existence of a set of expected I2C devices.
#

import os, sys, time, traceback
import subprocess
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.util import Util
from core.config_loader import ConfigLoader
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.screen import Screen
from hardware.monitor import Monitor
from hardware.irq_clock import IrqClock
import hardware.ThunderBorg3
from hardware.ThunderBorg3 import ThunderBorg, ScanForThunderBorg, SetNewAddress

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

EXPECT_GPS = False

_tb1 = None
_tb2 = None

try:

    _log = Logger('init', Level.INFO)

    _log.info("start...")

    _config = ConfigLoader(Level.INFO).configure()
    _screen = Screen(_config, Level.INFO)
#   _screen.disable()
    _i2c_scanner = I2CScanner(_config, level=Level.INFO)

    _log.info("scanning for thunderborgs...")
    ScanForThunderBorg()

    _tb1 = ThunderBorg(Level.INFO)
    _tb1.i2cAddress = 0x16
    _tb1.Init()

    _tb2 = ThunderBorg(Level.INFO)
    _tb2.i2cAddress = 0x15
    _tb2.Init()

    # thunderborg 1 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    if _tb1:
        _tb1.SetLedShowBattery(False)
        _tb1.SetLeds(0.0, 0.0, 0.0) # black
        pass

    # thunderborg 2 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    if _tb2:
        _tb2.SetLedShowBattery(False)
        _tb2.SetLeds(0.0, 0.0, 0.0) # black
        pass

    # check for expected devices ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
   
    # I²C address 0x0E: Digital Potentiometer: Panel
    if not _i2c_scanner.has_hex_address(['0x0E']):
        raise DeviceNotFound('Digital Potentiometer not found at address 0x0E.')
    # I²C address 0x0F: Digital Encoder
    if not _i2c_scanner.has_hex_address(['0x0F']):
        raise DeviceNotFound('Digital Encoder not found at address 0x0F.')
    # I²C address 0x10: PA1010D GPS
    if EXPECT_GPS and not _i2c_scanner.has_hex_address(['0x10']):
        raise DeviceNotFound('PA1010D GPS not found at address 0x10.')
    # I²C address 0x15: ThunderBorg: Fwd
    if not _i2c_scanner.has_hex_address(['0x15']):
        raise DeviceNotFound('Forward ThunderBorg not found at address 0x15.')
    # I²C address 0x16: ThunderBorg: Aft
    if not _i2c_scanner.has_hex_address(['0x16']):
        raise DeviceNotFound('Aft ThunderBorg not found at address 0x16.')
    # I²C address 0x19: IO Expander: Fwd
    if not _i2c_scanner.has_hex_address(['0x19']):
        raise DeviceNotFound('Forward IO Expander not found at address 0x19.')
    # I²C address 0x18: IO Expander: Aft
    if not _i2c_scanner.has_hex_address(['0x18']):
        raise DeviceNotFound('Aft IO Expander not found at address 0x18.')
    # I²C address 0x29: VL53L1X/VL53L5X
    if not _i2c_scanner.has_hex_address(['0x29']):
        raise DeviceNotFound('VL53L5X not found at address 0x29.')
    # I²C address 0x40: Servo Bonnet
    if not _i2c_scanner.has_hex_address(['0x40']):
        raise DeviceNotFound('Servo Bonnet not found at address 0x40.')
    # I²C address 0x41: INA260
    if not _i2c_scanner.has_hex_address(['0x41']):
        raise DeviceNotFound('INA260 not found at address 0x41.')
    # I²C address 0x48: ADS1015
    if not _i2c_scanner.has_hex_address(['0x48']):
        raise DeviceNotFound('ADS1015 not found at address 0x48.')
    # I²C address 0x69: ICM20948
    if not _i2c_scanner.has_hex_address(['0x69']):
        raise DeviceNotFound('ICM20948 not found at address 0x69.')
    # I²C address 0x74: 5x5 RGB Matrix
    if not _i2c_scanner.has_hex_address(['0x74']):
        raise DeviceNotFound('Starboard 5x5 RGB Matrix not found at address 0x74.')
    # I²C address 0x77: 5x5 RGB Matrix (or 11x7 LED Matrix)
    if not _i2c_scanner.has_hex_address(['0x77']):
        raise DeviceNotFound('Port 5x5 RGB Matrix not found at address 0x77.')
    # I²C address 0x75: 11x7 LED Matrix
    if not _i2c_scanner.has_hex_address(['0x75']):
        raise DeviceNotFound('11x7 RGB Matrix not found at address 0x75.')

    # only start monitor if not already running
    if not Util.already_running('monitor_exec.py'):
        _path = os.path.abspath("monitor_exec.py") 
        os.system(_path + ' &')
        _log.info('started monitor with keepalive.')
    else:
        _log.info('monitor already running.')

    _log.info("done.")

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting...')
except Exception as e:
    _log.error('{} thrown in thunderborg test: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    pass

#EOF
