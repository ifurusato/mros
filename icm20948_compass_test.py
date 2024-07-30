#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-05-27
#

import pytest
import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.orientation import Orientation
from core.config_loader import ConfigLoader
from hardware.rgbmatrix import RgbMatrix
from hardware.icm20948 import Icm20948

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
@pytest.mark.unit
def test_compass():

    _log = Logger('test', Level.INFO)

    try:

        # read YAML configuration
        _level = Level.INFO
        _config = ConfigLoader(_level).configure()

        _enable_port = True
        _enable_stbd = False
        _rgbmatrix = RgbMatrix(_enable_port, _enable_stbd, _level)
        _port_rgbmatrix = _rgbmatrix.get_rgbmatrix(Orientation.PORT)
        _icm20948 = Icm20948(_config, _port_rgbmatrix, _level)
        if not _icm20948.is_calibrated:
            _icm20948.calibrate()
        _icm20948.scan()

    except KeyboardInterrupt:
        _log.info('Ctrl-C caught; exiting…')
    except Exception as e:
        _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
    finally:
        pass

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():
    try:
        test_compass()
    except Exception as e:
        _log.error(Fore.RED + 'error in sound test: {}'.format(e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        pass

if __name__== "__main__":
    main()

#EOF
