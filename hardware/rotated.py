#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-08-06
# modified: 2024-08-06
#

import ioexpander as io
from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Rotated(object):
    '''
    Reads an analog pin from the aft IO Expander to determine if a patch
    of white electrical tape is under a Sharp IR sensor, indicating that
    the steering of the SAFT servo is in a rotated state.

    :param config:   the application configuration
    :param level:    the log level
    '''
    def __init__(self, config, level=Level.DEBUG):
        self._log = Logger('rotated', level)
        _cfg = config['mros'].get('hardware').get('rotated')
        _i2c_address    = _cfg.get('i2c_address')
        self._pin       = _cfg.get('pin')
        self._threshold = _cfg.get('threshold')
        self._ioe = io.IOE(i2c_addr=_i2c_address)
        self._ioe.set_adc_vref(3.3) # input voltage of IO Expander, this is 3.3 on Breakout Garden
        self._ioe.set_mode(self._pin, io.ADC)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def in_range(self):
        '''
        Returns True if the sensor is in range of the reflective surface
        on the rotated steering armature.
        '''
        return self._ioe.input(self._pin) > self._threshold

#EOF
