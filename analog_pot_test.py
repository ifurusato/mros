#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-06-08
# modified: 2024-06-08
#

import sys, time, traceback
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.config_loader import ConfigLoader
from hardware.analog_pot import AnalogPotentiometer

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    _log = Logger('test', Level.INFO)

    try:

        _config = ConfigLoader(Level.INFO).configure() 
        _analog_pot = AnalogPotentiometer(_config, Level.INFO)

        while True:
            _raw = _analog_pot.get_raw_value()
            _value = _analog_pot.get_value()
            _scaled = _analog_pot.get_scaled_value()
            _percent = _analog_pot.get_percentage_value()
            _log.info('raw: {:.2f}; value: {:d}; scaled: {:.2f}; {:d}%'.format(_raw, _value, _scaled, _percent))
            time.sleep(1)

    except KeyboardInterrupt:
        print('Ctrl-C caught; exiting…')
    except Exception as e:
        _log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
        traceback.print_exc(file=sys.stdout)
    finally:
        _log.info('complete.')

if __name__== "__main__":
    main()

#EOF
