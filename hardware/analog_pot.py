#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2024-06-08
# modified: 2024-06-08
#

from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level
from hardware.i2c_scanner import I2CScanner

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class AnalogPotentiometer(object):
    '''
    Wraps an IO Expander board as input for using a single pin to obtain a
    value from an analog potentiometer. This provides the raw value, a value
    between 0 and 330, between 0.0 and 1.0, and a value between 0 and 100.

    :param config:   the application configuration
    :param level:    the log level
    '''
    def __init__(self, config, level=Level.INFO):
        self._log = Logger('analog', level)
        if config is None:
            raise ValueError('no configuration provided.')
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _config = config['mros'].get('hardware').get('analog_potentiometer')
        self._i2c_address = _config.get('i2c_address')
        self._adc_pin = _config.get('adc_pin')
        self._log.info('analog potentiometer pin assignment: {:d};'.format(self._adc_pin))
        # configure IO Expander board
        _i2c_scanner = I2CScanner(config, level=level)
        if _i2c_scanner.has_address([self._i2c_address]):
            self._log.info('found IO Expander at address 0x{:02X}, configuring…'.format(self._i2c_address))
            try:
                self._log.info('instantiating IO Expander…')
                self._ioe = io.IOE(i2c_addr=self._i2c_address)
                self._ioe.set_mode(self._adc_pin, io.ADC)
                self._ioe.set_adc_vref(3.3)  # input voltage of IO Expander, this is 3.3v on Breakout Garden
                self._log.info('ready.')
            except ImportError:
                raise Exception('This script requires the pimoroni-ioexpander module\nInstall with: pip3 install --user pimoroni-ioexpander')
        else:
            raise Exception('no IO Expander found.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_ioe(self):
        return self._ioe

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_scaled_value(self):
        '''
        Return the analog value of the potentiometer, a float between 0.0 and 1.0.
        '''
        return (self._ioe.input(self._adc_pin) * 100.0) / 330.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_percentage_value(self):
        '''
        Return the analog value of the potentiometer, an int between 0 and 100.
        '''
        return int((self._ioe.input(self._adc_pin) * 100.0) / 3.3)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_value(self):
        '''
        Return the analog value of the potentiometer, an int between 0-330.
        '''
        return int(round(self._ioe.input(self._adc_pin) * 100.0))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def get_raw_value(self):
        '''
        Return the raw, unprocessed values from the IO Expander.
        '''
        return self._ioe.input(self._adc_pin)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_black(self):
        '''
        Noop. For compatibility with the DigitalPotentiometer.
        '''
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_rgb(self, value):
        '''
        Noop. For compatibility with the DigitalPotentiometer.
        '''
        pass

# EOF
