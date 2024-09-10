#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2024-05-23
#

import sys, time

try:
    import RPi.GPIO as GPIO
except Exception:
    print('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
    sys.exit(1)

import ioexpander as io

from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class PushButton(object):
    '''
    A simple pushbutton that can be configured to either use a GPIO pin or
    a pin on the IOExpander.

    Uses BCM mode, i.e., GPIO numbering, not pin numbers. When using the
    IOExpander the pin is on the IOExpander.

    This introduces a slight delay on returning a value in order
    to debounce the switch.

    :param config         the application configuration
    :param level:         the log level
    '''
    def __init__(self, config, level=Level.INFO):
        _cfg = config['mros'].get('hardware').get('push_button')
        self._pin = _cfg.get('pin')
        self._log = Logger('push-btn:{}'.format(self._pin), level)
        _source = _cfg.get('source') # either 'gpio' or 'ioe'
        if _source == 'gpio':
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self._ioe = None
            self._log.info('ready: pushbutton on GPIO pin {:d}'.format(self._pin))
        elif _source == 'ioe':
            _i2c_address = _cfg.get('i2c_address')
            self._ioe = io.IOE(i2c_addr=_i2c_address)
            self._ioe.set_mode(self._pin, io.IN_PU)
            self._log.info('ready: pushbutton on IO Expander pin {:d}'.format(self._pin))
        else:
            raise Exception('unrecognised source: {}'.format(_source))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pin(self):
        '''
        Returns the GPIO pin used by the pushbutton, using BCM numbering.
        '''
        return self._pin

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def pushed(self):
        '''
        Returns True if the button is pushed (low).
        '''
        if self._ioe:
            return self._ioe.input(self._pin) == 0
        else:
            _value = not GPIO.input(self._pin)
            time.sleep(0.1)
            return _value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        GPIO.cleanup(_pin)

#EOF
