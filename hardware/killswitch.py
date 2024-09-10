#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2021 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-08-18
# modified: 2021-08-18
#

from enum import Enum
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.event import Event
from core.component import Component

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class KillSwitch(Component):
    '''
    A simple monitor for a kill switch connected to a GPIO pin.

    Rather than any fancy message bus stuff we just set up a callback on the
    pin when enabled and just call a kill() method on KROS when triggered.

    This should be closed upon completion so the Pi resources are freed up.

    Lazily-imports and configures pigpio when the enabled.

    :param config:    the application configuration
    :param kros:      the KROS application
    :param level:     the loggin level
    '''
    def __init__(self, config, kros, level):
        self._log = Logger('kill', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(config)))
        self._config = config['kros'].get('hardware').get('killswitch')
        self._pin       = self._config.get('pin')
        self._kros      = kros
        self._pi        = None
        self._initd     = False
        self._triggered = False
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _callback_method(self, gpio, level, tick):
        if not self._triggered:
            self._triggered = True
            self._log.info('killswitch triggered on GPIO pin {}; logic level: {}; ticks: {}'.format(gpio, level, tick))
            self._kros.shutdown()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def reset(self):
        self._triggered = False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        Component.enable(self)
        if self.enabled:
            if not self._initd:
                try:
                    self._log.info('importing pigpio...')
                    import pigpio
                    # establish pigpio interrupts for kill switch
                    self._log.info('enabling killswitch...')
                    self._pi = pigpio.pi()
                    if not self._pi.connected:
                        raise Exception('unable to establish connection to Pi.')
                    self._pi.set_mode(gpio=self._pin, mode=pigpio.INPUT) # GPIO 12 as input
                    _cb1 = self._pi.callback(self._pin, pigpio.FALLING_EDGE, self._callback_method)
                    self._log.info('configured kill switch callback on pin {:d}.'.format(self._pin))
                except Exception as e:
                    self._log.warning('no kill switch available: error during configuration: {}'.format(e))
                finally:
                    self._initd = True
        else:
            self._log.warning('already enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        if self._pi:
            self._pi.stop()
        Component.close(self)
        self._log.info('closed.')

#EOF
