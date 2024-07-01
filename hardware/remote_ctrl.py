#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-04-05
# modified: 2024-06-02
#

import pigpio

from colorama import init, Fore, Style
init()

from core.event import Event
from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class RemoteControl(object):

    def __init__(self, config):
        self._log = Logger('remote', Level.INFO)
        _cfg = config['mros'].get('publisher').get('remote')
        # pin assignments
        self._d0_pin         = _cfg.get('d0_pin')
        self._d1_pin         = _cfg.get('d1_pin')
        self._d2_pin         = _cfg.get('d2_pin')
        self._log.info('remote control pin assignments:\t' \
                + Fore.RED   + ' d0={:d};'.format(self._d0_pin) \
                + Fore.BLUE  + ' d1={:d};'.format(self._d1_pin) \
                + Fore.GREEN + ' d2={:d}'.format(self._d2_pin))
        self._pi = None
        try:
            # establish pigpio interrupts for remote control pins
            self._log.info('enabling remote control interrupts…')
            self._pi = pigpio.pi()
            self._log.info('importing pigpio…')
            if not self._pi.connected:
                raise Exception('unable to establish connection to Pi.')
            self._pi.set_mode(gpio=self._d0_pin, mode=pigpio.INPUT) # data 0
            self._pi.set_mode(gpio=self._d1_pin, mode=pigpio.INPUT) # data 1
            self._pi.set_mode(gpio=self._d2_pin, mode=pigpio.INPUT) # data 2
            self._log.info('configuration complete.')
        except Exception as e:
            raise Exception ('{} thrown configuring remote input: {}'.format(type(e), e))
        finally:
            pass
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_selected_index(self):
        '''
        Return the raw selected index as an int.
        '''
        _index = int('{}{}{}'.format(self._pi.read(self._d2_pin), self._pi.read(self._d1_pin), self._pi.read(self._d0_pin)), 2)
        self._log.debug('remote: {}'.format(_index))
        return _index

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_remote(self):
        '''
            REMOTE_A: START
            REMOTE_B: BRAKE
            REMOTE_Y: STOP
            REMOTE_X: SHUTDOWN
            REMOTE_D: SLOW DOWN
            REMOTE_R: SLOW STARBOARD MOTOR (turn to starboard)
            REMOTE_U: SPEED UP
            REMOTE_L: SLOW PORT MOTOR (turn to port)
        '''
        _index  = self.get_selected_index()
        _number = _index + Event.REMOTE_A.num
        _event  = Event.from_number(_number)
        self._log.debug('event: {}'.format(_event))
        return _event

#EOF
