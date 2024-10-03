#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-03-27
# modified: 2024-05-23 #
# https://www.adafruit.com/product/4754
# https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085
# https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/report-types
#
import pytest
import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from core.cardinal import Cardinal
from hardware.rgbmatrix import RgbMatrix
from hardware.bno055 import BNO055

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
@pytest.mark.unit
def test_bno08x():

    try:

        _level = Level.INFO
        _log = Logger('bno055-test', _level)
        _config = ConfigLoader(Level.INFO).configure()
    
        _enable_port = False
        _enable_stbd = True
        _rgbmatrix = RgbMatrix(_enable_port, _enable_stbd, _level)
        _stbd_rgbmatrix = _rgbmatrix.get_rgbmatrix(Orientation.STBD)
    
        _bno055 = BNO055(_config, _stbd_rgbmatrix, _level)
        _bno055.calibrate()
    
        while True:
            _bno055_result = _bno055.read()
            _calibration = _bno055_result[0]
            _bno055_heading = _bno055_result[1]
            if _calibration.calibrated:
                if _bno055_heading is None or _calibration is None:
                    _log.info('could not determine heading.')
                else:
                    _log.info(Fore.CYAN + Style.BRIGHT + 'heading: {:>6.2f}°\t(bno055)'.format(_bno055_heading))
            time.sleep(1.0)

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except Exception as e:
        _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        pass

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    try:
        test_bno08x()
    except KeyboardInterrupt:
        print('done.')

if __name__== "__main__":
    main()

#EOF
