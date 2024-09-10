#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-08-01
#

import time
from threading import Thread
from colorama import init, Fore, Style
init()

import ioexpander as io

from core.logger import Logger, Level
from core.component import Component
from hardware.sound import Sound

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Player(Component):
    __instance = None
    IOE_I2C_ADDRESS = 0x18
    PIN_0 =  1
    PIN_1 =  2
    PIN_2 =  3
    PIN_3 =  4
    PIN_4 =  5
    PIN_5 =  6
    PIN_6 =  7
    PINS = [ PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6 ]
    '''
    A sound player that relies upon a connection between an IO Expander
    and an external sound module, which has five input pins, using BCD
    to trigger one of 128 sounds. The first sound (index 1) is a clip
    of silence, which is used to reset the player, which reacts only to
    changes.

    This is a singleton class; obtain its instance and play a sound via:

        Player.instance().play(Sound.CHIRP_1)

    The IO Expander pins are hardwired to avoid requiring configuration
    via a constructor.
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
            for pin in Player.PINS:
                cls.__instance._log.debug('configuring IOE output pin {} low.'.format(pin))
                cls.__instance._ioe.set_mode(pin, io.OUT)
                cls.__instance._ioe.output(pin, io.LOW)
            cls.__instance._looping = False
            cls.__instance._play_sound_thread = None
            cls.__instance._log.info('ready.')
        return cls.__instance

    def loop(self, sound, count):
        '''
        Plays one of the Sound enums in a loop.
        '''
        _index = sound.index
        _name = sound.name
        _duration = sound.duration
        self._log.info("looping sound '{}' with index {} for {} times…".format(_name, _index, count))
        for i in range(0, count):
            time.sleep(0.1)
            self._log.info("[{}]…".format(i))
            Player.play(_index)
            time.sleep(_duration)

    def stop(self):
        self._looping = False

    def continuous_loop(self, sound):
        '''
        Plays one of the Sound enums in a loop.
        '''
        _index = sound.index
        _name = sound.name
        _duration = sound.duration
        self._log.info("continuous looping sound '{}' with index {}…".format(_name, _index))
        self._looping = True
        _count = 0
        while self._looping:
            _count += 1
            time.sleep(0.1)
            self._log.info("[{}]…".format(_count))
            Player.play(_index)
            time.sleep(_duration)

    @staticmethod
    def int2bin(n):
        b = '{0:07b}'.format(n)
#       print("b: '{}'".format(b))
        _v0 = io.HIGH if b[0] == '1' else io.LOW
        _v1 = io.HIGH if b[1] == '1' else io.LOW
        _v2 = io.HIGH if b[2] == '1' else io.LOW
        _v3 = io.HIGH if b[3] == '1' else io.LOW
        _v4 = io.HIGH if b[4] == '1' else io.LOW
        _v5 = io.HIGH if b[5] == '1' else io.LOW
        _v6 = io.HIGH if b[6] == '1' else io.LOW
        return _v6, _v5, _v4, _v3, _v2, _v1, _v0

    @staticmethod
    def play_from_thread(value):
        '''
        Plays a Sound using a Thread, returning the sound duration. This is to
        avoid the sound interrupting normal operation.

        The argument can be a Sound or an index (0-31). Following play this sends
        a zero ('00000') to the TinyPICO as a reset, as it only reacts to changes.
        '''
        if isinstance(value, Sound):
            _sound = value
        elif isinstance(value, int):
            _sound = Sound.from_index(value)
        _player = Player.instance()
        _player.halt_thread()
        _is_daemon = True
        _player._play_sound_thread = Thread(target = _player._play, args=[_sound], name='play_sound', daemon=_is_daemon)
        _player._play_sound_thread.start()
        _duration = _sound.duration
        return _duration * 1.2

    @staticmethod
    def halt_thread():
        _player = Player.instance()
        if _player._play_sound_thread is not None:
            _player._log.info("halting thread…")
            _player._play_sound_thread.join()
            _player._log.info("halted thread.")

    def _play(self, sound):
        _index = sound.index
        _index -= 1 # zero-based
        _name = sound.name
        _duration = sound.duration
        _description = sound.description
        _b = Player.int2bin(_index)
        self._ioe.output(Player.PINS[0], _b[0])
        self._ioe.output(Player.PINS[1], _b[1])
        self._ioe.output(Player.PINS[2], _b[2])
        self._ioe.output(Player.PINS[3], _b[3])
        self._ioe.output(Player.PINS[4], _b[4])
        self._ioe.output(Player.PINS[5], _b[5])
        self._ioe.output(Player.PINS[6], _b[6])
        # reverse order in terms of BCD display
#       self._log.info(Fore.GREEN + "n={:d};\tb='{}{}{}{}{}{}{}'".format(_index, _b[6], _b[5], _b[4], _b[3], _b[2], _b[1], _b[0]))
        self._log.debug("playing sound [{}] '{}' ({}) for {:3.2f}s…".format(_index, _name, _description, _duration))
        # reset player by sending '00000'
        time.sleep(_duration)
        self._ioe.output(Player.PINS[0], io.LOW)
        self._ioe.output(Player.PINS[1], io.LOW)
        self._ioe.output(Player.PINS[2], io.LOW)
        self._ioe.output(Player.PINS[3], io.LOW)
        self._ioe.output(Player.PINS[4], io.LOW)
        self._ioe.output(Player.PINS[5], io.LOW)
        self._ioe.output(Player.PINS[6], io.LOW)
        time.sleep(0.1)
        self._play_sound_thread = None

    @staticmethod
    def play(value):
        '''
        Plays a Sound. The argument can be a Sound or an index (0-31).
        Following play this sends 0 ('00000') to the TinyPICO as a reset,
        as it only reacts to changes.
        '''
        if isinstance(value, Sound):
            _sound = value
        elif isinstance(value, int):
            _sound = Sound.from_index(value)
        _index = _sound.index
        _index -= 1 # zero-based
        _name = _sound.name
        _duration = _sound.duration
        _description = _sound.description
        _player = Player.instance()
        _b = Player.int2bin(_index)
        _player._ioe.output(Player.PINS[0], _b[0])
        _player._ioe.output(Player.PINS[1], _b[1])
        _player._ioe.output(Player.PINS[2], _b[2])
        _player._ioe.output(Player.PINS[3], _b[3])
        _player._ioe.output(Player.PINS[4], _b[4])
        _player._ioe.output(Player.PINS[5], _b[5])
        _player._ioe.output(Player.PINS[6], _b[6])
        # reverse order in terms of BCD display
#       _player._log.info(Fore.GREEN + "n={:d};\tb='{}{}{}{}{}{}{}'".format(_index, _b[6], _b[5], _b[4], _b[3], _b[2], _b[1], _b[0]))
        _player._log.debug("playing sound [{}] '{}' ({}) for {:3.2f}s…".format(_index, _name, _description, _duration))
        # reset player by sending '00000'
        time.sleep(_duration)
        _player._ioe.output(Player.PINS[0], io.LOW)
        _player._ioe.output(Player.PINS[1], io.LOW)
        _player._ioe.output(Player.PINS[2], io.LOW)
        _player._ioe.output(Player.PINS[3], io.LOW)
        _player._ioe.output(Player.PINS[4], io.LOW)
        _player._ioe.output(Player.PINS[5], io.LOW)
        _player._ioe.output(Player.PINS[6], io.LOW)
        time.sleep(0.1)
        _player._play_sound_thread = None

#EOF
