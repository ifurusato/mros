#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-25
# modified: 2024-05-25
#

from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Radar(object):
    '''
    A wrapper around the RCWL-0516 Microwave Radar Motion Sensor, using
    an IO Expander pin as input. This provides a digital output.

    Note than when triggered the output will remain high for 2-3 seconds
    before resetting.

    To be clear: the constructor argument is not a Raspberry Pi GPIO pin
    number, it is the pin on the IO Expander.

    :param config:   the application configuration
    :param level:    the log level
    '''
    def __init__(self, config, level=Level.DEBUG):
        self._log = Logger('radar', level)
        # configuration
        _config = config['mros'].get('hardware').get('radar')
        self._pin    = _config.get('pin')
        _i2c_address = _config.get('i2c_address')
        self._ioe    = io.IOE(i2c_addr=_i2c_address)
        self._ioe.set_adc_vref(3.3) # input voltage of IO Expander, this is 3.3 on Breakout Garden
        self._ioe.set_mode(self._pin, io.IN)  # digital input
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def triggered(self):
        '''
        Returns the current value as a property, True or False.
        '''
        return self._ioe.input(self._pin) 

#EOF
