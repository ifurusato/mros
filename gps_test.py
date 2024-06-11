#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-10-05
# modified: 2021-08-07
#
# Tests the port and starboard motors for directly by setting their power, from
# a digital potentiometer, without the intermediaries of velocity, slew, or PID
# controllers.
#

import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from hardware.gps import GPS

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    try:

        _gps = GPS(Level.INFO)
        _gps.set_verbose(True)
        
        while True:
            _log  = Logger('test', Level.INFO)
            _data = _gps.poll()
            if _data:
                _lat = _gps.latitude
                _long = _gps.longitude 
                _sats = _gps.number_of_satellites
                _qual = _gps.gps_quality
                _log.info(Fore.GREEN + 'lat-long: {:9.6f}, {:9.6f} from {} satellites, quality: {:.2f}.'.format(_lat, _long, _sats, _qual) + Style.RESET_ALL)
            else:
                _log.info(Fore.GREEN + Style.DIM + 'no data.' + Style.RESET_ALL)
            time.sleep(1)

    except Exception as e:
        print(Fore.RED + 'error in motor test: {}'.format(e) + Style.RESET_ALL)
        traceback.print_exc(file=sys.stdout)
    finally:
        pass

if __name__== "__main__":
    main()

#EOF
