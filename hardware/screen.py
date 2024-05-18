#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. 
# Released without license, this code is in the public domain.
#
# author:   Murray Altheim
# created:  2024-05-18
# modified: 2024-05-18
#
# The display screen, with an on and off control using a GPIO enable/disable pin.
#

import sys
try:
    import RPi.GPIO as GPIO
except Exception:
    print('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
    sys.exit(1)

from core.logger import Logger, Level
from core.component import Component

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Screen(Component):

    def __init__(self, config, level=Level.INFO):
#       super().__init__()
        self._log = Logger('screen', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['mros'].get('hardware').get('screen')
        self._channel = _cfg.get('channel')
        self._enabled = _cfg.get('enabled')
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._channel, GPIO.OUT)
        if self._enabled:
            self.enable()
        else:
            self.disable()
        self._log.info("ready.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        GPIO.output(self._channel, GPIO.HIGH)
        self._log.info("enabled.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        GPIO.output(self._channel, GPIO.LOW)
        self._log.info("disabled.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        GPIO.cleanup(self._channel)
        self._log.info("closed.")

#EOF
