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

    _log = Logger('thun', Level.INFO)
    _log.info("scanning for thunderborgs...")

    ScanForThunderBorg()

    _log.info("start...")

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

    # thunderborg 2 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    if _tb2:
        _tb2.SetLedShowBattery(False)
        _tb2.SetLeds(0.0, 0.0, 0.0) # black

    _log.info("done.")

except KeyboardInterrupt:
    _log.info('Ctrl-C caught; exiting...')
except Exception as e:
    _log.error('{} thrown in thunderborg test: {}\n{}'.format(type(e), e, traceback.format_exc()))
finally:
    pass

#EOF
