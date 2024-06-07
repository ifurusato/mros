#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-18
# modified: 2024-05-28
#
# Controls the enable/disable GPIO pin for the TFT display screen.
#

import sys
try:
    import RPi.GPIO as GPIO
except Exception:
    print('This script requires the RPi.GPIO module.\nInstall with: sudo pip3 install RPi.GPIO')
    sys.exit(1)
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Screen(object):

    def __init__(self, config, level=Level.INFO):
        super().__init__()
        self._log = Logger('screen', level)
        if config is None:
            raise ValueError('no configuration provided.')
        _cfg = config['mros'].get('hardware').get('screen')
        self._pin = _cfg.get('pin')
        self._log.info(Fore.WHITE + "configuring screen control on pin {}…".format(self._pin) + Style.RESET_ALL)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.OUT)
        if _cfg.get('enabled'):
            self.on()
        else:
            self.off()
        self._log.info("ready.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def on(self):
        self._log.info("enabling screen on channel {}…".format(self._pin))
        GPIO.setmode(GPIO.BCM)
        GPIO.output(self._pin, GPIO.HIGH)
        self._log.info("enabled.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def off(self):
        self._log.info("disabling screen on channel {}…".format(self._pin))
        GPIO.setmode(GPIO.BCM)
        GPIO.output(self._pin, GPIO.LOW)
        self._log.info("disabled.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        GPIO.cleanup(self._pin)
        self._log.info("closed.")

#EOF
