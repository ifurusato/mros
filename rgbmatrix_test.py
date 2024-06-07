#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-02-09
# modified: 2021-09-05
#

import pytest
import sys, time
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from core.orientation import Orientation
from hardware.i2c_scanner import I2CScanner
from hardware.color import Color
from hardware.rgbmatrix import RgbMatrix, DisplayType, WipeDirection

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def show_compass_calibration( rgbmatrix, orientation, s, g, a, m ):
    '''
    Set the four calibration status indicators based on four brightness values.
    The values in order are: system, gyroscope, accelerometer, and magnetometer.
    '''
    rgbmatrix.set_column(orientation, 0, Color.MAGENTA, s,  True)
    rgbmatrix.set_column(orientation, 1, Color.YELLOW, g, False)
    rgbmatrix.set_column(orientation, 2, Color.GREEN, a, False)
    rgbmatrix.set_column(orientation, 3, Color.CYAN, m, False)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
@pytest.mark.unit
def test_rgbmatrix():

    # which tests?
    _do_compass_cal   = True
    _do_headlights    = False
    _do_columns       = False
    _do_vertical_wipe = False
    _do_test_suite    = False

    _rgbmatrix = None

    try:

        _log = Logger("rgbmatrix-test", Level.INFO)

        # read YAML configuration
        _config = ConfigLoader(Level.INFO).configure()

        _i2c_scanner = I2CScanner(_config, Level.WARN)
        if not _i2c_scanner.has_address([0x74, 0x77]):
            _log.warning('test ignored: no rgbmatrix displays found.')
            return
        _addresses = _i2c_scanner.get_int_addresses()
        _enable_port = 0x77 in _addresses
        _enable_stbd = 0x74 in _addresses

        _rgbmatrix = RgbMatrix(_enable_port, _enable_stbd, Level.INFO)
        _log.info('starting test...')
        _port_rgbmatrix = _rgbmatrix.get_rgbmatrix(Orientation.PORT)
        _stbd_rgbmatrix = _rgbmatrix.get_rgbmatrix(Orientation.STBD)
  
        if _do_compass_cal:
            _off    = 0.0
            _dim    = 0.2
            _bright = 0.6

            show_compass_calibration(_rgbmatrix, Orientation.STBD, _bright, _dim, _dim, _dim )
            time.sleep(1)
            show_compass_calibration(_rgbmatrix, Orientation.STBD, _bright, _bright, _dim, _dim )
            time.sleep(1)
            show_compass_calibration(_rgbmatrix, Orientation.STBD, _bright, _bright, _bright, _dim )
            time.sleep(1)
            show_compass_calibration(_rgbmatrix, Orientation.STBD, _bright, _bright, _bright, _bright )
            time.sleep(1)

            show_compass_calibration(_rgbmatrix, Orientation.STBD, _dim, _bright, _bright, _bright )
            time.sleep(1)
            show_compass_calibration(_rgbmatrix, Orientation.STBD, _dim, _dim, _bright, _bright )
            time.sleep(1)
            show_compass_calibration(_rgbmatrix, Orientation.STBD, _dim, _dim, _dim, _bright )
            time.sleep(1)
            show_compass_calibration(_rgbmatrix, Orientation.STBD, _dim, _dim, _dim, _dim )
            time.sleep(1)

            show_compass_calibration(_rgbmatrix, Orientation.STBD, _off, _off, _off, _off )

        if _do_headlights:
            for v in range(0,255):
                _rgbmatrix.set_rgb_color(_port_rgbmatrix, v, v, v)
                _rgbmatrix.set_rgb_color(_stbd_rgbmatrix, v, v, v)
            while True:
                time.sleep(10)

        if _do_columns:
            # now the cylon scanning loop ......
            _rgbmatrix.clear(Orientation.CNTR)
            _log.info('starting column on ranged matrices, Ctrl-C to quit.')
            for i in range(10):
                for c in range(0,10):
                    _rgbmatrix.column(c)
                    time.sleep(0.001)
                for c in range(9,-1,-1):
                    _rgbmatrix.column(c)
                    time.sleep(0.001)

        if _do_vertical_wipe:
            _rgbmatrix.enable()
            # vertical wipe ....................................
            for c in [ Color.RED, Color.GREEN, Color.BLUE ]:
                _rgbmatrix.set_wipe_color(c)
                _rgbmatrix._wipe_vertical(_port_rgbmatrix, WipeDirection.DOWN)
                _rgbmatrix._wipe_vertical(_stbd_rgbmatrix, WipeDirection.DOWN)

        if _do_test_suite:
            # test suite .......................................
            _rgbmatrix.disable()
#           _types = [ DisplayType.CPU, DisplayType.SWORL, DisplayType.BLINKY, DisplayType.RAINBOW, DisplayType.RANDOM, DisplayType.WIPE_LEFT ]
            _types = [ DisplayType.RANDOM ]
            for display_type in _types:
                _log.info('rgbmatrix_test    :' + Fore.CYAN + Style.BRIGHT + ' INFO  : displaying {}...'.format(display_type.name))
                _rgbmatrix.set_display_type(display_type)
                _rgbmatrix.enable()
                time.sleep(5.0 if len(_types) == 1 else 2.0)
                _rgbmatrix.disable()
                count = 0
                while not _rgbmatrix.is_disabled():
                    count += 1
                    time.sleep(1.0)
                    if count > 5:
                        _log.info('rgbmatrix_test    :' + Fore.RED + Style.BRIGHT + ' INFO  : timeout waiting to disable rgbmatrix thread for {}.'.format(display_type.name))
                        sys.exit(1)
                _rgbmatrix.set_color(Color.BLACK)
                _log.info('{} complete.'.format(display_type.name))

        _log.info('test complete.')

    except KeyboardInterrupt:
        print('rgbmatrix_test    :' + Fore.YELLOW + ' INFO  : Ctrl-C caught: exiting...' + Style.RESET_ALL)
    finally:
        if _rgbmatrix:
            _rgbmatrix.disable()
            _rgbmatrix.close()

# main .........................................................................
def main():
    test_rgbmatrix()

if __name__== "__main__":
    main()

#EOF
