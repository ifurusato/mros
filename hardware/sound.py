#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-05-21
#

import time
from enum import Enum
from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level
from core.component import Component

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Sound(Enum):
    __order__ = " CHATTER_1 CHATTER_2 CHATTER_3 CHATTER_4 CHATTER_5 CHIRP_1 CHIRP_2 TELEMETRY KLAXON "
    #                 name         mnemonic    duration(s)  name         event                   file name
    CHATTER_1 = ( 0, 'chatter-1', 'CH1',       2 ) #        Chatter 1    some event              001-Chatter-1.mp3
    CHATTER_2 = ( 1, 'chatter-2', 'CH2',       1 ) #        Chatter 2    sensor hit              002-Chatter-2.mp3
    CHATTER_3 = ( 2, 'chatter-3', 'CH3',       1 ) #        Chatter 3    decision                003-Chatter-3.mp3
    CHATTER_4 = ( 3, 'chatter-4', 'CH4',       2 ) #        Chatter 4    confounded              004-Chatter-4.mp3
    CHATTER_5 = ( 4, 'chatter-5', 'CH5',       2 ) #        Chatter 5    confounded              005-Chatter-5.mp3
    CHIRP_1   = ( 5, 'chirp-1',   'CR1',       2 ) #        Chirp 1      thinking...             006-Chirp-01.mp3
    CHIRP_2   = ( 6, 'chirp-2',   'CR2',       2 ) #        Chirp 2      difficult thinking      007-Chirp-02.mp3 
    TELEMETRY = ( 7, 'telemetry', 'TEL',       1 ) #        Telemetry    encountered obstacle    008-Telemetry.mp3
    KLAXON    = ( 8, 'klaxon',    'KLX',       2 ) #        Klaxon       error                   009-Klaxon.mp3

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, mnemonic, duration):
        self._pin = num + 1
        self._name = name
        self._mnemonic = mnemonic
        self._duration = duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pin(self):
        return self._pin

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mnemonic(self):
        return self._mnemonic

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def duration(self):
        return self._duration

__player = None

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Player(Component):
    __instance = None
    IOE_I2C_ADDRESS = 0x18
    '''
    A sound player that relies upon a connection between an IO Expander
    and a YS-M3 Sound Module, which has nine input pins. Bring one of them
    low will trigger a specific sound from its SD card.

    This is a singleton class; obtain its instance and play a sound via:

        Player.instance().play(Sound.CHIRP_1)
    '''

    def __init__(self):
        raise RuntimeError('singleton: call instance() instead.')

    @classmethod
    def instance(cls):
        if cls.__instance is None:
            cls.__instance = cls.__new__(cls)
            # put any initialization here.
            cls.__instance._log = Logger('player', Level.INFO)
            Component.__init__(cls.__instance, cls.__instance._log, suppressed=False, enabled=True)
            cls.__instance._ioe = io.IOE(i2c_addr=Player.IOE_I2C_ADDRESS, interrupt_pin=4)
            for p in range(1, 10):
                cls.__instance._log.debug('setting IOE output pin {} high.'.format(p))
                cls.__instance._ioe.set_mode(p, io.OUT)
                cls.__instance._ioe.output(p, io.HIGH)
            cls.__instance._looping = False
            cls.__instance._log.info('ready.')
        return cls.__instance

    def loop(self, sound, count):
        '''
        Plays one of the Sound enums in a loop.
        '''
        _pin = sound.pin
        _name = sound.name
        _duration = sound.duration
        self._log.info("looping sound '{}' on pin {} for {} times…".format(_name, _pin, count))
        for i in range(0, count):
            time.sleep(0.1)
            self._log.info("[{}]…".format(i))
            self._ioe.output(_pin, io.LOW)
            time.sleep(_duration)
            self._ioe.output(_pin, io.HIGH)
            time.sleep(0.1)

    def stop(self):
        self._looping = False

    def continuous_loop(self, sound):
        '''
        Plays one of the Sound enums in a loop.
        '''
        _pin = sound.pin
        _name = sound.name
        _duration = sound.duration
        self._log.info("continuous looping sound '{}' on pin {}…".format(_name, _pin))
        self._looping = True
        _count = 0
        while self._looping:
            _count += 1
            time.sleep(0.1)
            self._log.info("[{}]…".format(_count))
            self._ioe.output(_pin, io.LOW)
            time.sleep(_duration)
            self._ioe.output(_pin, io.HIGH)
            time.sleep(0.1)

    @staticmethod
    def play(sound):
        '''
        Plays one of the Sound enums.
        '''
        _pin = sound.pin
        _name = sound.name
        _duration = sound.duration
        _player = Player.instance()
        _player._log.info("playing sound '{}' on pin {} for {}s…".format(_name, _pin, _duration))
        _player._ioe.output(_pin, io.LOW)
        time.sleep(_duration)
        _player._ioe.output(_pin, io.HIGH)

#EOF
